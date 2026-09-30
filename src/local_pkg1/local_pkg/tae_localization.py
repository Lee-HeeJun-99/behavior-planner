###2025년 7월 14일 수정본###
# get odometry and heading useing gps, imu, erp serial #

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import NavSatFix, Imu
from utm import from_latlon
from std_msgs.msg import Float64, Float32, Float32MultiArray
from geometry_msgs.msg import PointStamped
import math
from math import *
import numpy as np
import time
from scipy.signal import butter, lfilter_zi, lfilter, filtfilt
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class GpsImuHeading(Node):

    def __init__(self):
        super().__init__('tae_localization')
        UDP = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT)
        #--------------------------------------[ 설정 파라미터 ]------------------------------------------
        self.effective_velocity = 7 # gps헤딩 유효값 속도[km/h] (해당 속도부터 GPS로 imu값 보정)
        self.gps_heading_max_velocity = 25 # gps heading가 매우 유효한 속도[km/h] (해당 속도부터 GPS헤딩만 사용)
        self.gps_hz = 10.0 # GPS subscribe 속도[HZ]
        self.imu_hz = 200.0 # IMU subscribe 속도[HZ]
        self.publish_hz = 200.0 # publish 속도[HZ]
        self.odom_gps_error = 0.05 # erp odometry와 gps의 거리 에러값으로 gps 신뢰성 판단
        self.cutoff_freq = 5  # low pass filter 사용시 저역 통과 필터의 절단 주파수 설정 (Hz) [클 수록 높은 주파수도 통과]

        self.steering_filter_k = 0.5 # steering low pass filter 계수
        self.beta_filter_k = 0.70 # beta low pass filter 계수

        #--------------------------------------[ Pub and Sub ]-----------------------------------------
        self.gps_subscriber = self.create_subscription(NavSatFix, '/fix', self.gps_callback, UDP)
        self.erp_serial_subscriber = self.create_subscription(Float32MultiArray, '/ERP/serial_data', self.erp_serial_callback, UDP)
        self.imu_subscriber = self.create_subscription(Imu,'/imu',self.imu_callback, UDP)

        self.publish_timer = self.create_timer(1/self.publish_hz, self.timer_callback)
        self.beta_publisher = self.create_publisher(Float64,'/beta', UDP)
        self.utm_publisher = self.create_publisher(PointStamped, '/Local/utm', UDP)
        self.heading_publisher = self.create_publisher(Float64, 'Local/heading', UDP)        
        
        #--------------------------------- [ Check input Data  ] ---------------------------------------
        self.timer = self.create_timer(1.0, self.check_input_data)

        #--------------------------------------[ 멤버 변수 선언 ] ---------------------------------------
        self.first_run = True
        self.gps_on = False
        self.imu_on = False
        self.erp_serial_on = False

        self.corrections_imu_with_gps = False
        self.first_corrections = True
        
        self.gps_x = None
        self.gps_y = None
        self.gps_x_ = None
        self.gps_y_ = None

        self.gps_heading = None
        self.imu_heading = None
        self.imu_data_ = None
        self.imu_input_error_value = 10.0 # imu 이전값과 현재값 차이가 10도 이상 난다면 에러로 파악
        self.low_pass_filter_heading = None
        self.final_heading = None
        self.gps_effectiveness = 0.0


        self.gear = 0.0
        self.car_w = 0.985
        self.car_l = 1.04

        self.steering = None
        self.steering_ = None

        self.imu_input_heading = None
        self.imu_input_heading_ = None
        self.imu_input_error_data = 0.0

        self.correction_value = 0.0 # gps으로 imu보정값
        self.imu_up_and_down = 0
        self.correction_count = 0
        
        nyquist_freq = 0.5 * self.imu_hz
        normalized_cutoff = self.cutoff_freq / nyquist_freq
        self.b, self.a = butter(1, normalized_cutoff, btype='low', analog=False)
        self.zi = None

        self.left_velocity = None
        self.center_velocity = 0.0

        self.current_x = None
        self.current_y = None

        self.beta = None
        # self.estimated_beta = None

        self.tae_error_distance = 1000

        self.gps_tae_error_sum = 0.0

    def check_input_data(self):
        if self.erp_serial_on == False:
            self.get_logger().warning('ERP Serial data not entered')
        if self.gps_on == False:
            self.get_logger().warning('Gps data not entered')
        if self.imu_on == False:
            self.get_logger().warning('Imu data not entered')
        if self.erp_serial_on and self.gps_on and self.imu_on and self.first_run:
            permissible_error = atan2(0.02,(self.effective_velocity/3.6/self.gps_hz))*180/pi
            self.get_logger().info(f"🚨 permissible error for {self.effective_velocity}km/h : {permissible_error}° 🚨")
            self.first_run = False

        self.erp_serial_on = False
        self.gps_on = False
        self.imu_on = False


    def erp_serial_callback(self, msg):
        self.erp_serial_on = True
        self.gear = msg.data[2]
        self.left_velocity = msg.data[3]

        steering = msg.data[4]

        if self.steering_ is None:
            self.steering_ = steering

        self.steering = steering*self.steering_filter_k + self.steering_*(1-self.steering_filter_k)
        self.steering_ = self.steering


    def imu_callback(self, input_msg):
        self.imu_on = True
        self.get_odometry(input_msg)
        self.get_heading(input_msg)


    def gps_callback(self, msg):
        
        if np.isnan(msg.latitude) or np.isnan(msg.longitude) or np.isnan(msg.altitude):
            return
        else:
            self.gps_on = True

        utm_coords = from_latlon(msg.latitude, msg.longitude)
        self.correction_heading(utm_coords[0], utm_coords[1])


    def correction_heading(self, utm_x, utm_y):
        if self.current_x is not None:
            self.tae_error_distance = sqrt(pow(self.current_x - utm_x,2)+pow(self.current_y - utm_y,2))
            self.gps_tae_error_sum += self.tae_error_distance
            # print(self.gps_tae_error_sum)

        self.current_x = utm_x
        self.current_y = utm_y
        self.gps_x = utm_x
        self.gps_y = utm_y

        if self.gps_x_ is not None:
            gps_vector = atan2(self.gps_y - self.gps_y_, self.gps_x - self.gps_x_)
            self.beta = atan2(self.gps_y - self.gps_y_, self.gps_x - self.gps_x_)

        self.gps_x_ = self.gps_x
        self.gps_y_ = self.gps_y

        if(abs(self.center_velocity) >= self.effective_velocity/3.6 and self.tae_error_distance < self.odom_gps_error):
            self.correction(gps_vector)
            self.corrections_imu_with_gps = True
        else:
            self.corrections_imu_with_gps = False


    def get_odometry(self, imu_data):
        dt = 1/self.imu_hz

        yaw_rate = imu_data.angular_velocity.z
        # ax = -imu_data.linear_acceleration.x
        ay = imu_data.linear_acceleration.y
        az = imu_data.linear_acceleration.z
        left_velocity = self.left_velocity

        if left_velocity is not None and yaw_rate == 0:
            self.center_velocity = left_velocity

        elif left_velocity is not None and yaw_rate != 0:
            r_left = left_velocity/abs(yaw_rate)

            if yaw_rate > 0:
                r_right = r_left + self.car_w
            else:
                r_right = r_left - self.car_w

            right_velocity = abs(yaw_rate)*r_right

            self.center_velocity = (left_velocity + right_velocity)/2.0


        if self.center_velocity is not None and self.current_x is not None and self.beta is not None:
            if self.center_velocity < 0.1:
                pass
            else:
                beta_dot = ay/self.center_velocity + yaw_rate
                predicted = self.beta + beta_dot*dt
                self.beta = self.beta_filter_k*predicted + (1-self.beta_filter_k)*self.beta

                self.current_x = self.current_x + self.center_velocity * cos(self.beta) *dt
                self.current_y = self.current_y + self.center_velocity * sin(self.beta) *dt

    def get_heading(self, input_msg):
        if self.gps_heading != None:

            imu_resize_data = self.imu_input_data_resize(input_msg)

            self.imu_heading = self.get_correction_value(imu_resize_data)

            if self.zi is None:
                self.zi = lfilter_zi(self.b, self.a) * self.imu_heading

            # 필터링 적용
            self.low_pass_filter_heading, self.zi = lfilter(self.b, self.a, [self.imu_heading], zi=self.zi)

    def closer_to_zero(self, num1, num2):
        if abs(num1) < abs(num2):
            return num1
        else:
            return num2


    def find_imu_input_error(self, input_heading, input_heading_):
        
        if input_heading != None and input_heading_ != None:
            error = input_heading-input_heading_
            if abs(error) > self.imu_input_error_value and abs(error) < 360 - self.imu_input_error_value:
                self.imu_input_error_data = self.imu_input_error_data + error
                self.get_logger().error(f"imput imu data ERROR {error}")
            real_heading = input_heading - self.imu_input_error_data

            if real_heading <= -180:
                real_heading += 360
            elif real_heading >= 180:
                real_heading -= 360
        else:
            real_heading = input_heading

        return real_heading


    def timer_callback(self):
        if self.current_x is not None:
            utm_point = PointStamped()
            utm_point.header.stamp = self.get_clock().now().to_msg()
            utm_point.header.frame_id = 'map'
            utm_point.point.x = self.current_x
            utm_point.point.y = self.current_y

            self.utm_publisher.publish(utm_point)

        if self.beta is not None:
            beta_msg = Float64()
            beta_msg.data = float(self.beta)
            self.beta_publisher.publish(beta_msg)

        if(self.low_pass_filter_heading != None):

            heading_resize = self.low_pass_filter_heading[0]

            while (heading_resize < -180 or heading_resize > 180):
                if heading_resize >= 180:
                    heading_resize -= 360
                if heading_resize < -180:
                    heading_resize += 360

            self.final_heading = heading_resize * pi / 180

            heading_msg = Float64()
            heading_msg.data =  self.final_heading
            self.heading_publisher.publish(heading_msg)

    def imu_input_data_resize(self, input_msg):
        q_x, q_y, q_z, q_w = input_msg.orientation.x, input_msg.orientation.y, input_msg.orientation.z, input_msg.orientation.w
        self.imu_input_heading = self.quaternion_to_euler(q_x, q_y, q_z, q_w)*180/pi
        imu_data = self.find_imu_input_error(self.imu_input_heading, self.imu_input_heading_) # input된 imu data error 파악
        self.imu_input_heading_ = self.imu_input_heading

        if imu_data < 0:
            imu_data = imu_data + 360 # imu 값 범위를 0~360으로 설정
        elif imu_data > 360:
            imu_data = imu_data - 360

        if self.imu_data_ != None:
            heading_difference = imu_data - self.imu_data_
            
            if heading_difference < -(360 - self.imu_input_error_value):
                self.imu_up_and_down += 1
            elif heading_difference > (360 - self.imu_input_error_value):
                self.imu_up_and_down -= 1
        
        self.imu_data_ = imu_data
        imu_data = imu_data + 360 * self.imu_up_and_down

        return imu_data


    def get_correction_value(self, imu_data):
        if self.corrections_imu_with_gps == True:
            self.correction_count +=1
            # self.get_logger().info(f'corrections imu with gps {self.correction_count}: ')
            if self.first_corrections == True:
                self.get_logger().info('👍👍 corrections imu with gps 👍👍')
                self.first_corrections = False
                self.correction_value = self.gps_heading - imu_data
                self.corrections_imu_with_gps = False

            else:
                self.correction_value = self.gps_heading - imu_data - (self.gps_heading - self.imu_heading)*(1-self.gps_effectiveness)
                self.corrections_imu_with_gps = False

        return (imu_data + self.correction_value)


    def correction(self, gps_vector):
        effectiveness = self.center_velocity/self.gps_heading_max_velocity
        if effectiveness >= 1.0:
            effectiveness = 1.0
        self.gps_effectiveness = effectiveness

        heading = gps_vector*180/pi
        
        if self.gear == 2.0:
            # self.get_logger().info('back')
            if heading > 0:
                heading = heading - 180
            else:
                heading = heading + 180
        else:
            heading = heading

        if heading < 0:
            heading = heading + 360

        if self.first_corrections == True:
            self.gps_heading = heading
        else:
            
            updown = 0
            imu_resize = self.imu_heading

            while (imu_resize < 0 or imu_resize >= 360):
                if imu_resize >= 360:
                    imu_resize -= 360
                    updown += 1
                if imu_resize < 0:
                    imu_resize += 360
                    updown -= 1
            
            heading = heading + 360*updown
            
            if self.imu_heading - heading > 180:
                heading += 360
                
            elif heading - self.imu_heading > 180:
                heading -= 360

            self.gps_heading = heading
            # print("correct")


    def quaternion_to_euler(self, qx, qy, qz, qw):

        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = atan2(t3, t4)

        return yaw

def main(args=None):

    rclpy.init(args=args)
    node = GpsImuHeading()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()