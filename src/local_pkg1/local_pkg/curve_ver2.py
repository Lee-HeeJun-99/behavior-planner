#0912 0231 LAST CONFIRM
#일반적으로, 세 점 A, B, C가 주어졌을 때 B에서의 곡률은 다음과 같이 계산할 수 있습니다:
#BA와 BC라는 두 벡터를 구합니다.
#BA와 BC간의 내적으로부터 각도 θ를 찾습니다.
#|BA|와 |BC| (벡터 BA와 BC의 크기)로 평균 반경 r을 구합니다: r = (|BA| + |BC|)/2
#최종적으로 공식 K = 2sin(θ)/r 을 사용하여 공심각 θ에 대한 원호 길이를 찾아서 B에서 원호길이에 대한 변화율인 곡률 K를 얻게 됩니다.

import os
import glob
import shutil
import numpy as np
import matplotlib.pyplot as plt

# ROS2 library import and Node creation
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

path_dir='/home/'+os.getlogin()+'/ros2_ws/src/local_pkg/smooth_map/'
output_dir='/home/'+os.getlogin()+'/ros2_ws/src/local_pkg/curve_map/'
# 소수점 자릿수 설정
decimals = 8
# 최소치
small_number_threshold = 0.00000001   
#간격 0.05*20 = 1미터
# 60 = 3미터
# 100 = 5미터
interval = 60

class CurvatureCalculationNode(Node):
    def __init__(self):
        super().__init__('curvature_calculation')
        self.decimals = decimals
        self.small_number_threshold = small_number_threshold
        self.input_dir = path_dir
        self.output_dir = output_dir
        self.interval = interval

    def calculate_curvature(self):
        input_files = glob.glob(os.path.join(self.input_dir, '*.txt'))

        for input_path in input_files:
            with open(input_path, 'r') as f:
                lines = f.readlines()

            points_and_yaw = [list(map(float, line.strip().split(','))) for line in lines]
            points_and_yaw_array = np.array(points_and_yaw)

            # Initialize an array to hold curvature values.
            curvatures = np.zeros_like(points_and_yaw_array[:, 2])

            # Iterate over the points and yaw array with a step size equal to the given interval.
            for i in range(self.interval, len(points_and_yaw_array) - self.interval):
                three_points = points_and_yaw_array[[i - self.interval, i, i + self.interval]]

                points = three_points[:, :2]
                pathyaw_values = three_points[:, 2]

                vectors = np.diff(points, axis=0)
                norms = np.linalg.norm(vectors, axis=1)

                if np.any(norms < 1e-16):
                    print("Warning: Small norms detected. Replacing small norms with a small number.")
                    norms[norms < 1e-16] += 1e-16

                vectors /= norms[:, None]

                dot_products = np.sum(vectors[:-1] * vectors[1:], axis=1)

                distances = (norms[:-1] + norms[1:]) / 2

                dot_products = np.clip(dot_products, -1.0, 1.0)

                angles = np.arccos(dot_products)

                if np.any(distances < self.small_number_threshold):
                    print("Warning: Small distances detected. Replacing small distances with a small number.")
                    distances[distances < self.small_number_threshold] += self.small_number_threshold

                curvature = np.divide(2 * np.sin(angles), distances)
                
                # Check for extremely high curvature due to noise or very sharp turns and set them to zero.
                curvature[curvature > 100] = 0

                curvatures[i] = curvature   # 중간 점의 곡률을 저장


            output_filename_base = os.path.splitext(os.path.basename(input_path))[0]
            output_filename = output_filename_base + '.txt'
            output_path = os.path.join(self.output_dir, output_filename)

            # 백업용 코드
            if os.path.exists(output_path):
                backup_dir_base_name = 'Backup'
                i = 0

                while True:
                    i += 1
                    backup_dir_name = f'{self.output_dir}/{backup_dir_base_name}{i}'

                    if not os.path.exists(backup_dir_name):
                        break
                os.makedirs(backup_dir_name)

                if os.path.exists(output_path):
                    shutil.move(output_path, os.path.join(backup_dir_name, output_filename))

            with open(output_path, 'w') as f:
                for point, pathyaw_value, value in zip(points_and_yaw_array[:, :2], points_and_yaw_array[:, 2],
                                                       curvatures):
                    rounded_value = np.round(value, self.decimals)
                    if rounded_value < self.small_number_threshold:
                        rounded_value = 0

                    # Use string formatting to control how the number is displayed.
                    f.write(','.join(
                        map(str, list(point) + [pathyaw_value] + ["{:.{}f}".format(rounded_value, self.decimals)])) + '\n')

            self.get_logger().info(f"Processing completed for {input_path}!")

def main(args=None):
    rclpy.init(args=args)

    node = CurvatureCalculationNode()
    node.calculate_curvature()
    
    fig=plt.figure('Curvature Comparison')

    ax1=fig.add_subplot(211)  # Create the first subplot for path
    ax2=fig.add_subplot(212)  # Create the second subplot for curvature

    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

    input_files = glob.glob(os.path.join(node.output_dir, '*.txt'))

    curvatures_accumulated = []  # List to store all curvatures from different txt files
    indices_accumulated = []  # List to store indices of all curvatures

    for i, output_dir in enumerate(input_files):
        with open(output_dir, 'r') as f:
            lines = f.readlines()

        data = [list(map(float, line.strip().split(','))) for line in lines]
        data_array = np.array(data)

        x_values = data_array[:, 0]
        y_values = data_array[:, 1]
        curvatures = data_array[:, 3]

        ax1.plot(x_values, y_values, '-', label=f'{os.path.basename(output_dir)}', color=colors[i % len(colors)])

        start_index=len(curvatures_accumulated)
        
        # Extend the accumulated curvatures and indices lists with the current file's data
        curvatures_accumulated.extend(curvatures)
        
         # Generate index range considering previous accumulations.
        indices_range=list(range(start_index,start_index+len(curvatures)))
        
        indices_accumulated.extend(indices_range)
        

     # Plot all accumulated curvatures in order outside of loop.
    ax2.plot(indices_accumulated,curvatures_accumulated,'-',label='curvature_combined',color='green')
    
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax2.set_xlabel('Index')
    ax2.set_ylabel('Curvature')

    ax1.legend()
    ax2.legend()

    plt.show()

if __name__ == '__main__':
    main()
