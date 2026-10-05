#ifndef DATA_MANAGE_HPP
#define DATA_MANAGE_HPP

#include <cmath>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include "control/PIDController.hpp"

using namespace std;

class CallbackClass
{
private:
    // Local
    Point lo_odom;
    bool odom_sub_flag;
    double lo_yaw;
    double lo_yaw_rate;
    double lo_beta;

    // ERP Data (/ERP/serial_data)
    float control_mode;
    float e_stop;
    float gear;
    float speed;
    float steer;
    float brake;
    float enc;
    bool serial_sub_flag;

    vector<Point> pl_local_path;     // relative coordinate
    vector<Point> pl_local_path_abs; // abs coordinate
    vector<double> pl_local_path_yaw;
    int pl_mission_num;
    double pl_control_switch;

    int closest_index;
    bool path_too_far = false;
    double pridict_dist;
    double path_curvature = 0.0;
    double pre_path_curvature = 0.0;
    vector<Point> stanley_predict_pos;
    vector<float> stanley_predict_yaw;
    Point zero_point;

    Point AxisTrans_abs2rel(const Point &p)
    {
        Point rel;
        rel.x = (p.x - lo_odom.x) * cos(lo_yaw) + (p.y - lo_odom.y) * sin(lo_yaw);
        rel.y = -(p.x - lo_odom.x) * sin(lo_yaw) + (p.y - lo_odom.y) * cos(lo_yaw);
        return rel;
    }

public:
    CallbackClass()
    {
        // TODO: 생성자 작성 필요
        lo_odom.x = 0.0;
        lo_odom.y = 0.0;
        odom_sub_flag = false;
        lo_yaw = 0.0;
        lo_yaw_rate = 0.0;
        lo_beta = 0.0;

        control_mode = 0.0;
        e_stop = 0.0;
        gear = 0.0;
        speed = 0.0;
        steer = 0.0;

        brake = 0.0;
        enc = 0.0;
        serial_sub_flag = false;

        closest_index = 0;
        pridict_dist = 0.0;
        zero_point.x = 0.0;
        zero_point.y = 0.0;

        pl_control_switch = 0;
        pl_mission_num = -404; // 있을 수 없는 아무 숫자로 초기화해 줘야함.
    }

    ////////////////////////////////////////   local   ////////////////////////////////////////
    ////////////////////////////////////////   local   ////////////////////////////////////////
    ////////////////////////////////////////   local   ////////////////////////////////////////

    Point AxisTrans_rel2abs(const Point &p)
    {
        Point rel;
        rel.x = p.x * cos(lo_yaw) - p.y * sin(lo_yaw) + lo_odom.x;
        rel.y = p.x * sin(lo_yaw) + p.y * cos(lo_yaw) + lo_odom.y;
        return rel;
    }

    void lo_odom_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // info : [x, y], UTM coordinate
        this->lo_odom.x = msg->point.x;
        this->lo_odom.y = msg->point.y;
        this->odom_sub_flag = true;
    }

    void lo_yaw_cb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // info : radian, -pi ~ +pi
        this->lo_yaw = msg->data;
    }

    void lo_imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract yaw rate (z-axis angular velocity)
        this->lo_yaw_rate = msg->angular_velocity.z;
    }

    void lo_beta_cb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        this->lo_beta = msg->data;
    }

    ////////////////////////////////////////   planning   ////////////////////////////////////////
    ////////////////////////////////////////   planning   ////////////////////////////////////////
    ////////////////////////////////////////   planning   ////////////////////////////////////////

    void pl_local_path_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // [x0, y0, x1, y1, ...] 이므로 값이 없거나 홀수 개면 쓰지 않는다 (직전 경로 유지).
        if (msg->data.empty() || msg->data.size() % 2 != 0)
        {
            cerr << "local_path 길이가 잘못되었습니다 : " << msg->data.size() << endl;
            return;
        }
        vector<Point> points;
        vector<Point> points_abs;
        if (msg->data[0] == -82.82 || msg->data[1] == -82.82) // [-82.82, -82.82] -> 상대경로를 보내겠다는 시그널
        {
            for (size_t i = 2; i < msg->data.size(); i += 2)
            {
                Point point{msg->data[i], msg->data[i + 1]};
                points.push_back(point);
            }
        }
        else
        {
            // Convert the received message to a 2D vector
            for (size_t i = 0; i < msg->data.size(); i += 2)
            {
                Point point{msg->data[i], msg->data[i + 1]};

                points_abs.push_back(point);

                Point pl_rel = AxisTrans_abs2rel(point);
                points.push_back(pl_rel);
            }
            this->pl_local_path_abs = points_abs;
        }

        this->pl_local_path = points;
    }

    void pl_local_path_yaws_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.empty())
        {
            cerr << "path_yaw 길이가 0 입니다." << endl;
            return;
        }

        // 주의: 2번 칸부터 두 칸씩 건너뛰며 읽는다 (예전 코드 그대로).
        //       그래서 yaws[k] 는 경로의 (2 + 2k) 번째 점의 방향이다.
        //       지금 게인은 이 읽기 방식에 맞춰져 있으므로, 바꿀 때는 게인을 다시 맞춰야 한다.
        vector<double> yaws;

        for (size_t i = 2; i < msg->data.size(); i += 2)
        {
            yaws.push_back(msg->data[i]);
        }

        this->pl_local_path_yaw = yaws;
    }

    void pl_mission_num_cb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        // info : TODO!!!
        this->pl_mission_num = msg->data;
    }

    void pl_control_switch_cb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // info : -1 : REVERSE | 0 : STOP | 0.1 ~ 6.9 : DRIVE
        this->pl_control_switch = msg->data;
    }

    ////////////////////////////////////////   ERP   ////////////////////////////////////////
    ////////////////////////////////////////   ERP   ////////////////////////////////////////
    ////////////////////////////////////////   ERP   ////////////////////////////////////////

    void co_ERP_data_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // [control_mode, e_stop, gear, speed(m/s), steer(rad), brake(MPa), 0]
        if (msg->data.size() < 7)
        {
            return;
        }
        this->control_mode = msg->data[0];
        this->e_stop = msg->data[1];
        this->gear = msg->data[2];
        // ERP42 Pro 는 차량 속도를 준다 (브릿지가 실제 속도로 보정함).
        // 예전 ERP42 처럼 왼쪽 바퀴 속도를 차량 중심 속도로 환산하지 않는다.
        this->speed = std::isfinite(msg->data[3]) ? msg->data[3] : 0.0F;
        this->steer = msg->data[4];
        this->brake = msg->data[5];
        this->enc = msg->data[6];
        this->serial_sub_flag = true;
    }

    double calc_n_get_lat_error()
    {
        // 인자를 주지 않으면 이 함수가 호출
        // 인자를 주면 아래의 함수를 호출
        return calc_n_get_lat_error(this->zero_point);
    }

    double calc_n_get_lat_error(Point pridict_pose)
    {
        // 준비가 안 됐으면 404 를 돌려준다. 이유는 not_ready_reason() 으로 알 수 있다.
        if (pl_local_path.size() < 2 || lo_odom.x == 0.0 || lo_odom.y == 0.0)
        {
            return 404;
        }

        closest_index = 0; // 계산 결과로 나오는 값인데 이 값도 사용된다.
        double min_distance = std::sqrt(std::pow(pl_local_path[0].x - pridict_pose.x, 2) + std::pow(pl_local_path[0].y - pridict_pose.y, 2));

        for (size_t i = 1; i < pl_local_path.size(); i++)
        {
            double distance = std::sqrt(std::pow(pl_local_path[i].x - pridict_pose.x, 2) + std::pow(pl_local_path[i].y - pridict_pose.y, 2));
            if (distance < min_distance)
            {
                closest_index = static_cast<int>(i);
                min_distance = distance;
            }
        }

        if (min_distance > 50)
        {
            this->path_too_far = true;
            return 404; // 경로에서 50 m 넘게 떨어짐
        }
        this->path_too_far = false;

        // lat_error의 좌 우를 구분함. (가장 가까운 점이 경로의 마지막 점이면 그 앞 구간의 방향을 쓴다)
        int seg = std::min(closest_index, static_cast<int>(pl_local_path.size()) - 2);
        double l_or_r = determine_side(pl_local_path[seg + 1], pridict_pose, pl_local_path[seg]);
        if (l_or_r > 0)
        {
            min_distance = min_distance * -1;
        }
        return min_distance;
    }

    ////////////////////////////////////////   control_LQR   ////////////////////////////////////////
    ////////////////////////////////////////   control_LQR   ////////////////////////////////////////
    ////////////////////////////////////////   control_LQR   ////////////////////////////////////////

    double calc_path_curvature(float time_delay = 0.0, float diff_s = 1.5) // 1.5
    { 
        float td = clip(time_delay, 0.1F, 2.0F);
        pridict_dist = td * clip(this->speed, 2.5F, 9.0F); // (s)*(m/s)
        double second_pridict_dist = pridict_dist + diff_s;
        int f_pri_yaw_index = 0;
        int s_pri_yaw_index = 0;

        double s = 0.0;        // 경로의 길이 (Frenet s)
        double min_dist = 1e9; // 아주 큰 값으로 초기화
        double second_min_dist = 1e9;

        for (size_t i = 1; i < pl_local_path.size(); i++)
        {
            double dx = pl_local_path[i].x - pl_local_path[i - 1].x;
            double dy = pl_local_path[i].y - pl_local_path[i - 1].y;
            s += std::sqrt(dx * dx + dy * dy); // 경로의 길이 누적

            double f_dist = abs(s - pridict_dist);
            double s_dist = abs(s - second_pridict_dist);

            if (f_dist < min_dist)
            {
                f_pri_yaw_index = static_cast<int>(i);
                min_dist = f_dist;
            }
            if (s_dist < second_min_dist)
            {
                s_pri_yaw_index = static_cast<int>(i);
                second_min_dist = s_dist;
            }
        }
        if (pl_local_path_yaw.empty())
        {
            return path_curvature;
        }
        // path_yaw 를 읽는 방식 때문에 yaw 개수가 경로 점 개수보다 적다. 범위를 넘지 않게 한다.
        int last_yaw_index = static_cast<int>(pl_local_path_yaw.size()) - 1;
        f_pri_yaw_index = std::min(f_pri_yaw_index, last_yaw_index);
        s_pri_yaw_index = std::min(s_pri_yaw_index, last_yaw_index);

        double yaw_diff = - nomalize_angle(pl_local_path_yaw[f_pri_yaw_index] - pl_local_path_yaw[s_pri_yaw_index]);

        double tmp_curv = yaw_diff / diff_s;
        this->path_curvature = low_pass_filter(tmp_curv, pre_path_curvature, 0.9);
        this->pre_path_curvature = this->path_curvature;

        return path_curvature;
    }

    void calc_predict_odometry_for_stanley(float dt, int n)
    {
        vector<Point> predict_pos;

        double dx = cos(lo_beta - lo_yaw) * this->speed * dt;
        double dy = sin(lo_beta - lo_yaw) * this->speed * dt;

        for (int i = 1; i <= n; i++)
        {
            Point tmp_odom;

            tmp_odom.x = dx * i;
            tmp_odom.y = dy * i;
            predict_pos.push_back(tmp_odom);
        }

        vector<float> predict_yaw;

        for (int i = 1; i <= n; i++)
        {
            float tmp_yaw = this->lo_yaw + this->lo_yaw_rate * dt;
            predict_yaw.push_back(tmp_yaw);
        }

        this->stanley_predict_pos = predict_pos;
        this->stanley_predict_yaw = predict_yaw;
    }

    ////////////////////////////////////////   control_get   ////////////////////////////////////////
    ////////////////////////////////////////   control_get   ////////////////////////////////////////
    ////////////////////////////////////////   control_get   ////////////////////////////////////////

    // 제어를 시작할 수 없는 이유. 준비가 됐으면 nullptr.
    const char *not_ready_reason()
    {
        if (lo_odom.x == 0.0 || lo_odom.y == 0.0)
        {
            return "/Local/utm 을 아직 받지 못했습니다 (로컬라이제이션 확인)";
        }
        if (pl_local_path.size() < 2)
        {
            return "/Planning/local_path 를 아직 받지 못했습니다 (플래닝 확인)";
        }
        if (pl_local_path_yaw.empty())
        {
            return "/Planning/path_yaw 를 아직 받지 못했습니다 (플래닝 확인)";
        }
        if (path_too_far)
        {
            return "경로에서 50 m 넘게 떨어져 있습니다 (맵 확인)";
        }
        return nullptr;
    }

    Point get_odom()
    {
        return this->lo_odom;
    }

    bool get_odom_sub_flag()
    {
        return this->odom_sub_flag;
    }

    void set_down_odom_sub_flag()
    {
        this->odom_sub_flag = false;
    }

    double get_yaw()
    {
        return this->lo_yaw;
    }

    float get_speed()
    {
        return this->speed;
    }

    float get_steer()
    {
        return this->steer;
    }

    double get_pd_path_yaw()
    {
        if (this->pl_local_path_yaw.empty())
        {
            return 404;
        }
        // path_yaw 를 읽는 방식 때문에 yaw 개수가 경로 점 개수보다 적다. 범위를 넘지 않게 한다.
        size_t index = std::min(static_cast<size_t>(this->closest_index), this->pl_local_path_yaw.size() - 1);
        return this->pl_local_path_yaw[index];
    }

    int get_mission_num()
    {
        return this->pl_mission_num;
    }

    double get_control_switch()
    {
        return this->pl_control_switch;
    }

    vector<double> get_local_path_yaw()
    {
        return this->pl_local_path_yaw;
    }

    float get_yawrate()
    {
        return this->lo_yaw_rate;
    }

    double get_path_curvature()
    {
        return this->path_curvature;
    }

    bool get_serial_sub_flag()
    {
        return this->serial_sub_flag;
    }

    void set_down_serial_sub_flag()
    {
        this->serial_sub_flag = false;
    }

    vector<Point> get_relative_path()
    {
        return this->pl_local_path;
    }

    vector<Point> get_stanley_predict_pos()
    {
        return this->stanley_predict_pos;
    }

    vector<float> get_stanley_predict_yaw()
    {
        return this->stanley_predict_yaw;
    }
};

#endif // DATA_MANAGE_HPP