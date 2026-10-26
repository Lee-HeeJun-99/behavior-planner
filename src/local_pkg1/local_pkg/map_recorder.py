###2023년 8월 15일 수정본###
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from datetime import datetime
from math import *
import shutil
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile
import string
import os, time
from geometry_msgs.msg import PointStamped
import sys, tty, select, termios

import numpy as np

import random
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import glob
#os.gelogin()

from pathlib import Path
TARGET_SAVE_PATH = '/home/wgr/local_pkg_2025-main/map/261009'


GPS_TOPIC = '/Local/utm'

class MapRecorder(Node):
    def __init__(self):
        super().__init__('map_recorder')
        
        self.declare_parameter('save_path', TARGET_SAVE_PATH)
        self.save_path = os.path.join(
            os.path.expanduser(self.get_parameter('save_path').value),
            datetime.now().strftime('record_%Y%m%d_%H%M%S_%f'))
        os.makedirs(self.save_path, exist_ok=False)
        self.get_logger().info(f'Save directory: {self.save_path}')
        self.lock = threading.RLock()
        self.finalized = False
        self.save_mode = 0
        self.callback_counter = 0
        self.settings = None
        self.utm = np.empty((0, 2), dtype=np.float64)
        self.total_saved = []
        self.cx, self.cy = [], []
        self.pre_x, self.pre_y = 0, 0

        self.topic_check_time = time.time()

        self.idx = 0
        self.sub = self.create_subscription(PointStamped, GPS_TOPIC, self.text_callback, qos_profile_sensor_data)

        # count files in specific location
        file_list = [f for f in os.listdir(self.save_path) if os.path.isfile(os.path.join(self.save_path, f))]

        if len(file_list)!=0:

            #in order to determine the title of backup folder to first item of existing files.
            abs_path = os.path.join(self.save_path,file_list[0])
            new_folder_path = os.path.join(self.save_path,self.get_file_time(abs_path))
            self.get_logger().warn(f'since there are files in save path, executing backup created to : {new_folder_path}')
            if not os.path.exists(new_folder_path):
                os.mkdir(new_folder_path)

            # breakpoint()
            for f in file_list:
                shutil.move(os.path.join(self.save_path, f), os.path.join(new_folder_path,f))

        
        self.get_logger().info("#### #### MAP SAVER #### ####\n")
        self.get_logger().info("writing x,y for UTM")
        self.get_logger().info("P: save segment and continue; S / Ctrl+C: save and stop")


        self.exit_event = threading.Event()
        input_thread = threading.Thread(target=self.get_input, args=(self.exit_event,self.idx,))
        input_thread.daemon = True
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            input_thread.start()
        
    def get_file_time(self, filepath):

        tm = os.path.getmtime(filepath)
        dt = datetime.fromtimestamp(tm)

        now = datetime.now()
        return dt.strftime("back_%y%m%d_%H%M%S")

    def get_input(self, exit_event,idx):
        # Set stdin to non-blocking mode
    
        while not exit_event.is_set():
            # Check if there is any input waiting on stdin
            if select.select([sys.stdin], [], [], 0.1)[0]:
                # Read a single character of input from stdin
                user_input = sys.stdin.read(1)
    
                # Do something with the user input (e.g. print it)
                # print("You entered:", user_input)
                if user_input == 'p' or user_input == 'P':
                    with self.lock:
                        if len(self.utm):
                            self.total_saved.append(self.utm)
                            self.utm = np.empty((0, 2), dtype=np.float64)
                            self.idx += 1
                            self.save_to_txt()
                        else:
                            self.get_logger().warn('No points in the current segment.')

                elif user_input == 's' or user_input == 'S':
                    exit_event.set()
                    break
    
            if time.time() - self.topic_check_time > 3:
                if self.callback_counter == 0:
                    self.get_logger().warn(f'i think The topic \'{GPS_TOPIC}\' I\'m subscribing is not that publishing anything. please check the topic again.')
                    self.topic_check_time = time.time()
        return

    def text_callback(self, msg):
        self.callback_counter = self.callback_counter+1
        target_utm = np.array([msg.point.x,msg.point.y])
    
        print(f'\r recording : {self.idx +1}.txt |  {target_utm}', end='')
        with self.lock:
            self.utm = np.append(self.utm, [target_utm], axis=0)

    def save_to_txt(self):
        for i, l in enumerate(self.total_saved):
            # self.get_logger().error(l)
            np.savetxt(os.path.join(self.save_path,f'{i+1}.txt'), l, delimiter=',', fmt='%.8f')
        self.get_logger().info('save done.')

    def plot(self, save_fig = True):            
        for i, u in enumerate(self.total_saved):
            random.seed(time.time())
            r = random.random()
            g = random.random()
            b = random.random()
            random_rgb=(r,g,b)

            x_start = u[0,0]
            x_end = u[-1,0]
            y_start = u[0,1]
            y_end = u[-1,1]

            plt.plot(u[:,0], u[:,1], c=random_rgb, marker= '.' ,markersize=6)
            #start point
            if i == 0:
                plt.text(x_start, y_start, "Start", fontsize=13, color='r')

            #end point
            elif i == len(self.total_saved)-1:
                plt.text(x_end, y_end, "End", fontsize=13, color='r')

            plt.text(np.mean(u[:,0]),np.mean(u[:,1]), "{}.txt".format(str(i+1)), fontsize=13)

        tm = time.localtime(time.time())
        string = time.strftime('%Y_%m_%d_ %I_%M_%S_%p', tm)
        plt.title(string)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis("equal")
        plt.grid(True)

        plt.savefig(os.path.join(self.save_path,f'{string}.png'),dpi=300)
        # PNG is saved without blocking on a plot window.

    def final_process(self):
        if self.finalized:
            return
        with self.lock:
            if len(self.utm):
                self.total_saved.append(self.utm)
                self.utm = np.empty((0, 2), dtype=np.float64)
            self.save_to_txt()
            if self.total_saved:
                self.plot(save_fig=True)
            self.finalized = True
        self.get_logger().info('#### #### PROGRAM END #### ####')


def main(args=None):
    rclpy.init(args=args)

    mr = MapRecorder()

    try:
        while rclpy.ok():            
            rclpy.spin_once(mr, timeout_sec=0.1)

            if mr.exit_event.is_set():
                # exit event 발생 시 최종 프로세스 처리
                mr.final_process()
                break

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    except Exception as e:
        mr.get_logger().error(f"An error ocurred: {str(e)}")
        raise e

    finally:
        # restore terminal
        mr.exit_event.set()
        if mr.settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, mr.settings)
        mr.final_process()
        plt.close('all')
        mr.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
