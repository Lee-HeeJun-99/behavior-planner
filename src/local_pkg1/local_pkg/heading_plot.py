###2023년 8월 15일 수정본###
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import math
from sensor_msgs.msg import NavSatFix
from utm import from_latlon

class HeadingPlot(Node):
    def __init__(self):
        super().__init__('heading_plot')
        self.heading_subscriber = self.create_subscription(Float64, '/Local/heading', self.heading_callback, qos_profile_sensor_data)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/fix', self.gps_callback, qos_profile_sensor_data)

        self.heading_array = []
        self.heading_array_index = 300 #그래프에 표시할 heading 점갯수


        self.gps_array_index = 1000 #그래프에 표시할 GPS점 갯수
        self.x_array = []
        self.y_array = []
        self.current_x = -1.0
        self.current_y = -1.0
        self.first_gps = True
        self.gps_distance = 0.01 # gps 이전 점과 현재 점 사이 간격을 어느정도 할것인지. (m단위)


    def heading_callback(self, msg):
        if(len(self.heading_array) > self.heading_array_index):
                self.heading_array.pop(0)
        self.heading_array.append(msg.data * 180/math.pi)

    def gps_callback(self,msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        utm_coords = from_latlon(lat, lon)
        easting = utm_coords[0]
        northing = utm_coords[1]
        distance = math.sqrt(math.pow(northing-self.current_y,2)+math.pow(easting-self.current_x,2))
        # print("distance : ", distance)

        if (self.current_x != easting and self.current_y != northing) and distance >= self.gps_distance:
            
            self.heading = math.atan2(northing-self.current_y,easting-self.current_x)

            if(len(self.x_array) >= self.gps_array_index):
                self.x_array.pop(0)
                self.y_array.pop(0)

            self.x_array.append(easting)
            self.y_array.append(northing)
                
            # print("heading : ", self.heading*180/math.pi)
            
        self.current_x = easting
        self.current_y = northing


def spin_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = HeadingPlot()

    
    try:
        thread = threading.Thread(target=spin_node, args=(node,))
        thread.start()
        rate = node.create_rate(10)

        fig, (ax1, ax2) = plt.subplots(2, 1)

        print("Waiting for GPS data")

        while rclpy.ok():                

            if(len(node.x_array) >= 1):

                ax1.cla()
                
                ax1.plot(node.x_array, node.y_array, marker='o', linestyle='--', color='blue', label='Pose')
                
                if(len(node.heading_array)>=1): #make car
                    heading_x_1 = -0.3*math.cos(node.heading_array[-1]*math.pi/180) + node.x_array[-1]
                    heading_y_1 = -0.3*math.sin(node.heading_array[-1]*math.pi/180) + node.y_array[-1]
                    heading_x_2 = 0.3*math.cos(node.heading_array[-1]*math.pi/180) + node.x_array[-1]
                    heading_y_2 = 0.3*math.sin(node.heading_array[-1]*math.pi/180) + node.y_array[-1]
                    ax1.plot([heading_x_1,heading_x_2], [heading_y_1, heading_y_2], linestyle='-', color='green', linewidth = "10")
                    ax1.plot(heading_x_2, heading_y_2, marker='o', color='red')

                ax1.set_xlabel('UTM X')
                ax1.set_ylabel('UTM Y')
                ax1.set_title('Plot GPS UTM')
                ax1.legend(loc="upper left")

                ax2.cla()
                if(len(node.heading_array)>=1):
                    ax2.plot(range(len(node.heading_array)), node.heading_array, linestyle='-', color='blue', label='Heading')

                    ax2.set_ylim(-180, 180)
                    ax2.set_xlabel('Time')
                    ax2.set_ylabel('Heading')
                    ax2.set_title('Heading over Time')
                    ax2.legend(loc="upper left")

                plt.tight_layout()
                plt.draw()
                plt.pause(0.05)

            # rate.sleep()

        thread.join()  # 스레드가 종료될 때까지 기다림

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
