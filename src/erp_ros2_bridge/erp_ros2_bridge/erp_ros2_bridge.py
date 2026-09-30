#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

import serial
import struct
import math
import time


"""
ERP 진짜 이상한게 읽기는 리틀 엔디안으로 읽어야 하는데
쓰기는 빅엔디안으로 써야 함..
A팀 ERP 기준 일단.
"""

DEFAULT_SERIAL_PATH = "/dev/ttyUSB0"
#/dev/ttyUSB0

class SerialRosBridge(Node):

    def __init__(self):
        super().__init__('erp_ros2_bridge')

        self.initialize_data()

        self.declare_parameter('port', DEFAULT_SERIAL_PATH)
        serial_path = self.get_parameter('port').get_parameter_value().string_value

        # Set up the serial port
        self.serial = serial.Serial(serial_path, baudrate=115200)
        self.buffer = bytearray()

        # Set up qos
        UDP = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT)
        # qos_profile = 1

        # Create publishers for the ROS topics
        self.ERP_data_pub_ = self.create_publisher(Float32MultiArray,
                                                   '/ERP/serial_data',
                                                   UDP)

        # Create subscription for Float32MultiArray topic
        self.ERP_data_sub_ = self.create_subscription(Float32MultiArray,
                                                      "/Control/serial_data",
                                                      self.erp_data_callback,
                                                      UDP)
        
        self.timer = self.create_timer(0.02, self.read_serial_data)
        # self.timer = self.create_timer(0.02, self.write_serial_data)


    def initialize_data(self):
        # Reading data
        self.control_mode_r_ = 0.0
        self.e_stop_r_ = 0.0
        self.gear_r_ = 0.0
        self.speed_r_ = 0.0
        self.steer_r_ = 0.0
        self.brake_r_ = 0.0
        self.enc_r_ = 0.0

        # Transmitting data
        self.control_mode_t_ = 0x01
        self.e_stop_t_ = 0x00
        self.gear_t_ = 0
        self.speed_t_ = 0
        self.steer_t_ = 0
        self.brake_t_ = 0
        self.alive_t_ = 0


    def erp_data_callback(self, msg):

        tmp_speed = min(max(msg.data[3], 0), 6.9)
        tmp_steer = min(max(msg.data[4], -0.491642), 0.491642)
        tmp_brake = min(max(msg.data[5], 1), 199)


        self.control_mode_t_ = int(msg.data[0])
        self.e_stop_t_ = int(msg.data[1])
        self.gear_t_ = int(msg.data[2])
        self.speed_t_ = int(tmp_speed * 36)  # m/s to Km/h*10
        self.steer_t_ = -int(tmp_steer * 71 * (180 / 3.14159))
        self.brake_t_ = int(tmp_brake)  # 1~200


        self.get_logger().info(f"SUB data :\n{list(msg.data)}")

    def write_serial_data(self):

        # 데이터를 바이트로 패킹
        packed_data = struct.pack('> 3B H h BB',
                                  self.control_mode_t_,
                                  self.e_stop_t_,
                                  self.gear_t_,
                                  self.speed_t_,
                                  self.steer_t_,
                                  self.brake_t_,
                                  self.alive_t_)

        # STX와 \r\n 추가
        final_data = b'\x53\x54\x58' + packed_data + b'\x0D\x0A'


        # Update alive counter
        self.alive_t_ += 1
        self.alive_t_ %= 256

        self.serial.write(final_data)

        # for i in final_data:
        #     self.get_logger().debug(hex(i))
        # for i in final_data:
        #     self.get_logger().debug(str(int(i)))
    

    def pub_serial_data(self):
        erp_data_msg = Float32MultiArray()
        erp_data_msg.data.extend([self.control_mode_r_,
                                  self.e_stop_r_,
                                  self.gear_r_,
                                  self.speed_r_ / 36.0,
                                  self.steer_r_ / 71 * (math.pi / 180),
                                  self.brake_r_,
                                  self.enc_r_])

        self.ERP_data_pub_.publish(erp_data_msg)


    def process_data(self, raw_data):
        # Process the received data here
        for j in range(len(self.buffer) - 3):
            if raw_data[j : j+3] == bytearray([0x53, 0x54, 0x58]):  # STX
                if len(raw_data) > j + 17:
                    # Unpack the relevant data using struct
                    self.control_mode_r_, self.e_stop_r_,  \
                        self.gear_r_, self.speed_r_,  \
                        self.steer_r_, self.brake_r_, self.enc_r_  \
                            = struct.unpack("< 3B H h B I", raw_data[j+3:j+15])
                            # b: char(4bit), B: unsigned char(4bit)
                            # h: short(8bit), H: unsigned short(8bit)
                            # i, I: int, uint (16bit)

                    # publish ros topic
                    self.pub_serial_data()

                    # receive buffer를 비움
                    del raw_data[j : j+18]
                    break
            else:
                self.get_logger().warn("지금 이거 잘못되어서 나오는 데이터임")
                self.get_logger().warn(str(list(raw_data)))


    def read_serial_data(self):
        
        self.write_serial_data()

        while self.serial.in_waiting > 0:
            incoming_byte = self.serial.read(1)
            self.buffer.extend(incoming_byte)

            # \r\n 패턴을 찾음
            if self.buffer[-2:] == b'\r\n':
                if False:  # for debug
                    self.get_logger().info("Received: %s" % [int(b) for b in self.buffer])
                    # self.get_logger().info("Received(hex): %s" % [hex(b) for b in self.buffer])

                self.process_data(self.buffer)

                # 버퍼를 비움
                self.buffer = bytearray()
                


def main(args=None):
    rclpy.init(args=args)
    node = SerialRosBridge()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
