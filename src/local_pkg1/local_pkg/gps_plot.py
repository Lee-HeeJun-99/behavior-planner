###2023년 8월 15일 수정본###
# subscribe gps fix data and plot for utm

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from utm import from_latlon
import matplotlib.pyplot as plt
import threading
import math

class GpsSubscriber(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.gps_subscriber = self.create_subscription(NavSatFix, '/fix', self.subscribe_gps_message, qos_profile_sensor_data)

        self.heading = -1.0
        self.current_x = -1.0
        self.current_y = -1.0
        self.x_array = []
        self.y_array = []
        self.array_index = 500 #그래프에 표시할 점 갯수
        self.gps_distance = 0.03 # gps 이전 점과 현재 점 사이 간격을 어느정도 할것인지. (m단위)

    def subscribe_gps_message(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        utm_coords = from_latlon(lat, lon)
        easting = utm_coords[0]
        northing = utm_coords[1]
        distance = math.sqrt(math.pow(northing-self.current_y,2)+math.pow(easting-self.current_x,2))
        print("distance : ", distance)

        if (self.current_x != easting and self.current_y != northing) and distance >= self.gps_distance:
            
            self.heading = math.atan2(northing-self.current_y,easting-self.current_x)

            if(len(self.x_array) >= self.array_index):
                self.x_array.pop(0)
                self.y_array.pop(0)

            self.x_array.append(easting)
            self.y_array.append(northing)
                
            # print("heading : ", self.heading*180/math.pi)
            

        self.current_x = easting
        self.current_y = northing


        # print(utm_coords)


def spin_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = GpsSubscriber()

    try:
        thread = threading.Thread(target=spin_node, args=(node,))
        thread.start()
        rate = node.create_rate(20)

        while rclpy.ok():

            if(len(node.x_array) >= 1):

                plt.clf()  # 그래프 영역을 지우고 새로운 점을 그릴 준비
                plt.plot(node.x_array, node.y_array, marker='o', linestyle='--', color='blue')  # 모든 점을 파란색으로 그리기
                plt.plot(node.x_array[-1], node.y_array[-1], marker='o', color='red')  # 마지막 점을 빨간색으로 그리기
                plt.xlabel('UTM X')
                plt.ylabel('UTM Y')
                plt.title('Plot GPS UTM')
                
                plt.draw()  # 그래프 업데이트
                plt.pause(0.001)  # 잠시 일시정지하여 그래프를 표시
                

            rate.sleep()

        plt.show()  # 그래프 표시

        thread.join()  # 스레드가 종료될 때까지 기다림
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
