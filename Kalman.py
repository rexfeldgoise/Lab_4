from XRPLib.differential_drive import DifferentialDrive
from machine import Timer
import time, math, gc, os
from XRPLib.board import Board
from XRPLib.defaults import *
drivetrain = DifferentialDrive.get_default_differential_drive()


#Motor and IMU setup

left_motor = EncodedMotor.get_default_encoded_motor(index=1)
right_motor = EncodedMotor.get_default_encoded_motor(index=2)
board = Board.get_default_board()
imu = IMU.get_default_imu()
hardware_timer_period = 0.1  # s

FK_pos = []
kf_pos = []



class PositionEstimation:
    def __init__(self):

        self.x = self.y = self.theta = 0
        self.kf_x = self.kf_y = self.kf_theta = 0
        self.prev_kf_theta = 0
        self.w = self.w_kf = 0

        self.track_width = 15.5
        self.wheel_diameter = 6
        self.RPMtoCMPS = (math.pi* self.wheel_diameter) / 60
        self.CMPStoRPM = 60 / (math.pi*self.wheel_diameter)
        self.innovation = 0


        self.P = [
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.0, 0.1]
        ]

        self.Q = [
            [0.01, 0.0, 0.0],
            [0.0, 0.01, 0.0],
            [0.0, 0.0, 0.01]
        ]

        self.R = [
            [0.05, 0.0, 0.0],
            [0.0, 0.05, 0.0],
            [0.0, 0.0, 0.01]
        ]


    def matrix_add(self,A, B):
        return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    

    #To filter noisy IMU data
    def median_filt(self):
        
        data = []
        for i in range(5):
            temp = imu.get_gyro_z_rate() * (math.pi / 180000)
            data.append(temp)

        data.sort()
        self.w_kf = data[2]
        

    def k_filt(self):

        
        
        #Prediction 
        self.prev_kf_theta = self.kf_theta
        self.median_filt()

        theta_z = self.kf_theta + self.w_kf * hardware_timer_period
        theta_x = self.prev_kf_theta + self.w * hardware_timer_period
        innovation = (theta_z-theta_x)
        self.P = self.matrix_add(self.P,self.Q)


        # Update 
        P_theta = self.P[2][2]
        R_theta = self.R[2][2]
        K = P_theta/(P_theta + R_theta)

        self.kf_theta = theta_x + K*innovation
        self.P[2][2] = (1-K)*self.P[2][2]
  

    def update_both(self):


        left_speed = left_motor.get_speed() * self.RPMtoCMPS
        right_speed = right_motor.get_speed() * self.RPMtoCMPS 
        self.w = (right_speed - left_speed) / self.track_width

        #Updates coordiantes for FK model
        if self.w == 0:
            V = 0.5 * (left_speed + right_speed)
            self.x = self.x + V * math.cos(self.theta) * hardware_timer_period
            self.y = self.y + V * math.sin (self.theta) * hardware_timer_period

        else:
            R_val = self.track_width * (right_speed + left_speed) / (2 * (right_speed - left_speed))
            self.x = self.x + -R_val * math.sin(self.theta) + R_val * math.sin(self.theta + self.w * hardware_timer_period)
            self.y = self.y + R_val * math.cos(self.theta) - R_val * math.cos(self.theta + self.w * hardware_timer_period)
            self.theta = self.theta + self.w * hardware_timer_period

        
        #updates kalman gain and theta
        self.k_filt()


        #Updates coordiantes for kalman_filter
        
        if self.w == 0:
            V = 0.5 * (left_speed + right_speed)
            self.kf_x = self.kf_x + V * math.cos(self.prev_kf_theta) * hardware_timer_period
            self.kf_y = self.kf_y + V * math.sin (self.prev_kf_theta) * hardware_timer_period

        else:
            R_val = self.track_width * (right_speed + left_speed) / (2 * (right_speed - left_speed))
            self.kf_x = self.kf_x + -R_val * math.sin(self.prev_kf_theta) + R_val * math.sin(self.kf_theta)
            self.kf_y = self.kf_y + R_val * math.cos(self.prev_kf_theta) - R_val * math.cos(self.kf_theta)

        
        #Adds FK and kalman filter data 
        FK_pos.append([self.x,self.y])
        kf_pos.append([self.kf_x,self.kf_y])
    
    def set_motor_target_speeds(self,left_cmps,right_cmps):
        left_motor.set_speed(left_cmps * self.CMPStoRPM)
        right_motor.set_speed(right_cmps * self.CMPStoRPM)


    
    #Set's up a trajectory with sharp turns and straight
    def execute_trajectory(self):

        motion_sequence = [

            (0, 0, 1), (20, 20, 5), (-100, 100, 5), (20, 20, 4), (0, 0, 0)
        ]

        for left, right, duration in motion_sequence:
            self.set_motor_target_speeds(left, right)
            time.sleep(duration)

    
    
    #Runs the XRP trajectory and calculates the state estimates
    def run_system(self):
        timer = Timer()
        self.display = False
        board.wait_for_button()

        timer.init(period=int(hardware_timer_period * 1000),
            mode=Timer.PERIODIC, 
            callback=lambda t: self.update_both())
        

        self.execute_trajectory()
   
kinematics = PositionEstimation()

    
kinematics.run_system()

print(FK_pos)
print("")
print("")
print("")
print(kf_pos)