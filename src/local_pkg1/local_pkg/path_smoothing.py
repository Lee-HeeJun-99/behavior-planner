#!/usr/bin/env python3

from math import *
import os, sys
import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
import rclpy
from rclpy.node import Node
from multiprocessing import Process


# 0911_06:05 last version
weight_data = 0.01 #0.01 0.04 ysê°€ ê³ ì¹¨ ã…ˆã……
weight_smooth = 0.20
tolerance = 0.000001

# #sungwook for control tune
# weight_data = 0.01
# weight_smooth = 0.05
# tolerance = 0.0
path_dir='/home/'+os.getlogin()+'/ros2_ws/src/local_pkg/map/'
out_path_dir='/home/'+os.getlogin()+'/ros2_ws/src/local_pkg/smooth_map/'

class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')
        self.yaw_ = None
        self.yaw = None
        self.min_distance = 0.01

    def smooth(self, path, weight_data, weight_smooth, tolerance):
        newpath = [[0 for col in range(len(path[0]))] for row in range(len(path))]
        
        for i in range(len(path)):
            for j in range(len(path[0])):
                newpath[i][j] = path[i][j]

        change = 1
        while change > tolerance:
            change = 0
            for i in range(1,len(path)-1):
                for j in range(len(path[0])):
                    ori = newpath[i][j]
                    newpath[i][j] += weight_data * (path[i][j]-newpath[i][j])
                    newpath[i][j] += weight_smooth * (newpath[i+1][j]+newpath[i-1][j]-2*newpath[i][j])
                    change += abs(ori - newpath[i][j])

        return newpath 


    def interpolate_path(self, path, spacing=0.05):
        x_coords=[coord[0]for coord in path]
        y_coords=[coord[1]for coord in path]

        distance=np.cumsum(np.sqrt(np.ediff1d(x_coords,to_begin=0)**2+np.ediff1d(y_coords,to_begin=0)**2))

        interpolator_x=interpolate.interp1d(distance,x_coords)
        interpolator_y=interpolate.interp1d(distance,y_coords)

        distance_new=np.arange(0,distance[-1],spacing)

        return list(zip(interpolator_x(distance_new),interpolator_y(distance_new)))

    def calculate_yaw(self, coords):
        yaws=[]
        for i in range(len(coords)-1):
            dx=coords[i+1][0]-coords[i][0]
            dy=coords[i+1][1]-coords[i][1]
            if (dx < self.min_distance or dy < self.min_distance)and len(coords) -6 > i:
                for k in range(5):
                    dx = coords[i+k+2][0]-coords[i][0]
                    dy = coords[i+k+2][1]-coords[i][1]
                    if dx > self.min_distance and dy > self.min_distance:
                        break
            
            
            self.yaw = atan2(dy,dx)

            if self.yaw_ == None :
                self.yaw_ = self.yaw
            
            self.yaw = self.yaw*0.95 + self.yaw_*0.05
            
            self.yaw_ = self.yaw

            yaws.append(self.yaw)
        yaws.append(yaws[-1]) # Append the last yaw as it is same as second last 
        return yaws

def main(args=None):
    rclpy.init(args=args)

    node = PathSmoother()

    file_list=os.listdir(path_dir)
    txts=[file for file in file_list if file.endswith(".txt")]

    fig=plt.figure('Path Smoothing Comparison')

    ax1=fig.add_subplot(211)  # Create the first subplot for path
    ax2=fig.add_subplot(212)  # Create the second subplot for yaw

    colors=['b','g','r','c','m','y','k']

    # Get user input for GET_YAW variable 
    user_input = input("Do you want yaw values? (Y/N): ")
    GET_YAW = True if user_input.lower() == 'y' else False

    yaws_accumulated = []  # List to store all yaws from different txt files

    for i ,txt in enumerate(txts):
        node.yaw = None
        node.yaw_ = None
        print(txt,'is changeing')
        with open(os.path.join(path_dir ,txt), 'r') as f:
            lines=f.readlines()
            coords=[list(map(float,line.strip().split(',')))for line in lines]
            old_coords=coords.copy() 
            smoothed_coords=node.smooth(coords, weight_data, weight_smooth, tolerance) 

            interpolated_coords=node.interpolate_path(smoothed_coords)
            
            if GET_YAW == True:
                yaws = node.calculate_yaw(interpolated_coords)
                yaws_accumulated.extend(yaws)  # Add current yaws to the accumulated list

            old_x ,old_y=zip(*old_coords)
            interpolated_x , interpolated_y=zip(*interpolated_coords)
            ax1.plot(old_x , old_y ,'-',label=f'{txt}',color='red')
            ax1.plot(interpolated_x , interpolated_y ,'-',label=f'result_{txt}',color=colors[(i+1)%len(colors)])

        scale_factor = 0.5  # Adjust this value to change the arrow length

        # if GET_YAW == True:
        #     for coord, yaw in zip(interpolated_coords, yaws):
        #         ax1.arrow(coord[0], coord[1], scale_factor*cos(yaw), scale_factor*sin(yaw), color='red', head_width=0.01)

        
        base_name=os.path.splitext(txt)[0]
        save_file=os.path.join(out_path_dir,f'{base_name}.txt')
        
        if GET_YAW == True:
            with open(save_file,'w') as f:
                for coord, yaw in zip(interpolated_coords, yaws):
                    f.write(f'{coord[0]},{coord[1]},{yaw}\n')
        else:
            with open(save_file,'w') as f:
                for coord in interpolated_coords:
                    f.write(f'{coord[0]},{coord[1]}\n')

    
    indices_accumulated = range(len(yaws_accumulated))
    ax2.plot(indices_accumulated, yaws_accumulated, label='yaw_combined', color='blue')

    ax1.set_xlabel('x')
    ax1.set_ylabel('y')

    ax1.legend()
    ax2.legend()

    # plot_process = Process(target=plt.show)
    # plot_process.start()
    # plot_process.join()  # Wait for the process to finish

if __name__ == '__main__':
    main()