# -*- coding: utf-8 -*-
"""
Created on Fri Sep 27 10:23:33 2024

@author: HZH

Servo Tracking using Usb Camera and Bus Servo
"""

import math
import serial
import cv2

import os
#current_path = os.path.dirname(os.path.abspath(__file__))
#os.chdir(current_path)
os.chdir(r'/Users/huangzehao/Desktop/Project/[科研] Pupil_Size_Test/Data_Exp_FIles/Exp_Scripts_add_Screen_marker_202503/Eye_SImulator')
from uservo import UartServoManager

class BusServo:
    def __init__(self, com_port, center_angle=None, speed=50):
        self.port = com_port
        self.center_angle = center_angle if center_angle else [16, -16, 8, 8]
        self.speed = speed
        self.uservo = None
        self.center_marker = [0,0,0,0]
        self.current_angle = [0,0,0,0]
        self.angle_limits =  {
                            0: (-22, 44),
                            1: (-50, 4),
                            2: (-15, 34),
                            3: (-21, 55)
                            }
        
        self.backlash = {0:-0.6, 1:0.6, 2:0.6, 3:-1}
        self.last_direction = [1, 1, 1, 1] # 1:left; -1:right
        
    def init_server(self):
        uart = serial.Serial(
            port=self.port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=0
        )
        self.uservo = UartServoManager(uart, is_debug=True)
        print(f'Bus Server started at {self.port}')

    def recenter(self):
        for i in range(4):
            self.uservo.set_servo_angle(i, self.center_angle[i], self.speed)
        self.current_angle = self.center_angle[:]
        print('Eye direction reset complete')
        
    def recenter_marker(self, angle):
        for i in range(4):
            tar_angle = self.center_angle[i] + angle[i]
            self.uservo.set_servo_angle(i, tar_angle, self.speed)
            self.center_marker[i] = tar_angle

    def send_servo_angles(self, angle):
        
        for i in range(4):
            
            if angle[i] == 0:
                pass
            else:
                tar_angle = self.center_angle[i] + angle[i]
                
                # backlash compensation for reverse motion
                current = self.current_angle[i]
                
                # direction detection
                if tar_angle - current >= 0:
                    direction = 1
                else:
                    direction = -1
                
                if abs(tar_angle - current) >= 2:
               
                    if direction != self.last_direction[i]:
                        if direction == 1:
                            tar_angle += self.backlash[i]
                        elif direction == -1:
                            tar_angle -= self.backlash[i]
                
                # update last direction
                self.last_direction[i] = direction
                
                # angle limition for protect the eyeholder
                lower, upper = self.angle_limits[i]
                tar_angle = max(lower, min(upper, tar_angle))
                        
                # # servo moving
                # step = 1 if tar_angle > current else -1
                # end_angle = int(tar_angle + step)
                
                # for step_angle in range(current, end_angle, step):
                #     self.uservo.set_servo_angle(i, step_angle, self.speed)
                #     self.current_angle[i] = step_angle
                
                self.uservo.set_servo_angle(i, tar_angle, self.speed)
                self.current_angle[i] = tar_angle
            

class CameraHandler:
    def __init__(self, camera_index=1, marker_size=3.5, ipd=6.5, tar_dis=66, zcamera=5.5):

        self.marker_size = marker_size
        self.ipd = ipd
        self.tar_dis = tar_dis
        self.zcamera = zcamera
        
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        # load the predefined ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.parameters = cv2.aruco.DetectorParameters()

        if not self.camera.isOpened():
            raise ValueError("Camera initialization failed")
        print("Camera initialized successfully")

    def capture_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            raise ValueError("Failed to capture frame")
        return frame

    def detect_marker(self, frame):
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters = self.parameters
        )

        if ids is not None:
            ids = ids.flatten()
            
            if ids[0] ==24:
                for marker_corner, marker_id in zip(corners, ids):
                    corners = marker_corner.reshape((4,2))
                    (top_left, top_right, bottom_right, bottom_left) = corners
        
                    # compute marker center
                    center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                    center_y = int((top_left[1] + bottom_right[1]) / 2.0)
        
                    # compute frame center
                    image_center_x = frame.shape[1] // 2
                    image_center_y = frame.shape[0] // 2
        
                    # calculate distances
                    distance_x = center_x - image_center_x
                    distance_y = center_y - image_center_y
        
                    marker_len_pix = math.sqrt((top_left[0] - top_right[0]) ** 2 +
                                               (top_left[1] - top_right[1]) ** 2)
        
                    pix2cm = self.marker_size / marker_len_pix
        
                    return distance_x * pix2cm, distance_y * pix2cm
                return None

    def calculate_angles(self, cood_x, cood_y):
        # Calculate X angles for the left servo
        if cood_x >= 0:
            alpha4 = math.degrees(math.atan((self.ipd / 2 + cood_x) / self.tar_dis))
            alpha3 = math.degrees(math.atan((self.ipd / 2) / self.tar_dis))
            alpha2 = abs(alpha4) - abs(alpha3)
        elif cood_x < 0 and cood_x > -self.ipd / 2:
            alpha4 = math.degrees(math.atan((self.ipd / 2 - abs(cood_x)) / self.tar_dis))
            alpha3 = math.degrees(math.atan((self.ipd / 2) / self.tar_dis))
            alpha2 = -(abs(alpha3) - abs(alpha4))
        elif cood_x <= -self.ipd / 2:
            alpha5 = math.degrees(math.atan((abs(cood_x) - self.ipd / 2) / self.tar_dis))
            alpha3 = math.degrees(math.atan((self.ipd / 2) / self.tar_dis))
            alpha2 = -(abs(alpha5) + abs(alpha3))
        deg_x_l = round(alpha2, 1)

        # Calculate X angles for the right servo
        alpha3_r = math.degrees(math.atan((self.ipd / 2) / self.tar_dis))
        if cood_x < 0:
            alpha4_r = math.degrees(math.atan((self.ipd / 2 + abs(cood_x)) / self.tar_dis))
            alpha2_r = - (alpha4_r - alpha3_r)
        elif cood_x >= 0 and cood_x < self.ipd / 2:
            alpha4_r = math.degrees(math.atan((self.ipd / 2 - cood_x) / self.tar_dis))
            alpha2_r = alpha3_r - alpha4_r
        elif cood_x >= self.ipd / 2:
            alpha4_r = math.degrees(math.atan((cood_x - self.ipd / 2) / self.tar_dis))
            alpha2_r = alpha4_r + alpha3_r
        deg_x_r = round(alpha2_r, 1)

        # Calculate Y angles
        beta3 = math.degrees(math.atan(self.zcamera / self.tar_dis))
        if cood_y > 0:
            beta4 = math.degrees(math.atan((self.zcamera + cood_y) / self.tar_dis))
            beta2 = beta4 - beta3
        elif cood_y <= 0 and cood_y > -self.zcamera:
            beta4 = math.degrees(math.atan((self.zcamera - abs(cood_y)) / self.tar_dis))
            beta2 = -(beta3 - beta4)
        elif cood_y <= -self.zcamera:
            beta4 = math.degrees(math.atan((abs(cood_y) - self.zcamera) / self.tar_dis))
            beta2 = - (beta3 + beta4)
        deg_y = round(beta2, 1)

        return deg_x_r*0.5, -deg_y, deg_y, deg_x_l*0.5
    
    def release_camera(self):
        self.camera.release()
        cv2.destroyAllWindows()

def main():
    cam_handler = CameraHandler(camera_index=0)
    servo = BusServo(com_port="/dev/cu.usbserial-110")
    servo.init_server()
    
    servo.recenter()

    try:
        while True:
            frame = cam_handler.capture_frame()
            marker_position = cam_handler.detect_marker(frame)
            
            if marker_position:
                cood_x, cood_y = marker_position
                angles = cam_handler.calculate_angles(cood_x, cood_y)
                print("Servo angles:", servo.current_angle)
                servo.send_servo_angles(angles)

            # display the frame for visualization
            cv2.imshow("Camera Feed", frame)
           
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    except Exception as e:
        print("Error:", e)

    finally:
        servo.recenter()
        cam_handler.release_camera()
        
        
if __name__ == '__main__':
    main()
    
    
   