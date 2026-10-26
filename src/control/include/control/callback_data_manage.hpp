#ifndef DATA_MANAGE_HPP
#define DATA_MANAGE_HPP

#include <algorithm>
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
    double yaw_lookahead_m_ = 0.0; // 경로 방향을 가장 가까운 점보다 이만큼 앞에서 읽는다 [m] (게인 파일)

    // 위치 튐 보호 (10/9 추가). /Local/utm 이 차가 움직일 수 없을 만큼 갑자기 바뀌면 그만큼을 offset 으로 빼 두고
    // pos_jump_tau_ 초의 시간 상수로 천천히 0 으로 줄인다. 로컬라이제이션의 GPS 튐 처리가 놓친 것을 막는 안전장치.
    Point raw_odom_prev_{0.0, 0.0};
    double raw_odom_stamp_prev_ = -1.0;
    bool has_raw_odom_prev_ = false;
    Point pos_jump_offset_{0.0, 0.0};
    int pos_jump_count_ = 0;
    // [m] 한 메시지 사이에 예상보다 (pos_jump_gate_ + pos_jump_gate_per_speed_ x 속도[m/s]) 넘게 움직이면 튄 것으로 본다.
    //     (10/5, 10/9 기록에서 튐이 아닐 때 한 메시지의 최대 어긋남은 8.4 cm (7.2 km/h). 빠를수록 추측항법 오차가 커서 속도에 비례해 늘린다)
    double pos_jump_gate_ = 0.10;
    double pos_jump_gate_per_speed_ = 0.03;
    double pos_jump_max_ = 2.0;     // [m] 이보다 크게 튀면 (재초기화 등) 보호하지 않고 그대로 따라간다
    double pos_jump_tau_ = 1.0;     // [s] offset 을 줄이는 시간 상수
    double pos_jump_active_ = 0.03; // [m] offset 이 이보다 크면 '보호 중' (Stanley 적분을 멈춘다)
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
        Point raw{msg->point.x, msg->point.y};

        // 메시지 사이 시간 (로컬라이제이션이 찍은 시각, 없으면 200 Hz 로 본다)
        double stamp = static_cast<double>(msg->header.stamp.sec) + 1e-9 * static_cast<double>(msg->header.stamp.nanosec);
        double dt = 0.005;
        if (stamp > 0.0 && raw_odom_stamp_prev_ > 0.0)
        {
            dt = clip(stamp - raw_odom_stamp_prev_, 0.001, 0.1);
        }
        raw_odom_stamp_prev_ = stamp;

        if (has_raw_odom_prev_)
        {
            // 예상 이동 (지금 속도와 헤딩) 과 실제 이동의 차이
            double jx = (raw.x - raw_odom_prev_.x) - this->speed * cos(lo_yaw) * dt;
            double jy = (raw.y - raw_odom_prev_.y) - this->speed * sin(lo_yaw) * dt;
            double jump = std::hypot(jx, jy);
            double gate = pos_jump_gate_ + pos_jump_gate_per_speed_ * std::abs(this->speed);
            if (jump > gate && jump < pos_jump_max_)
            {
                pos_jump_offset_.x += jx;
                pos_jump_offset_.y += jy;
                pos_jump_count_++;
                std::cout << "[위치 튐] /Local/utm 이 " << jump * 100.0 << " cm 튐 -> " << pos_jump_tau_
                          << " 초 시간 상수로 천천히 따라감 (" << pos_jump_count_ << " 번째)" << std::endl;
            }
            else if (jump >= pos_jump_max_)
            {
                pos_jump_offset_ = {0.0, 0.0}; // 너무 크면 그대로 따라간다
            }
        }
        raw_odom_prev_ = raw;
        has_raw_odom_prev_ = true;

        // offset 을 시간 상수 pos_jump_tau_ 로 줄인다
        double decay = std::exp(-dt / pos_jump_tau_);
        pos_jump_offset_.x *= decay;
        pos_jump_offset_.y *= decay;

        this->lo_odom.x = raw.x - pos_jump_offset_.x;
        this->lo_odom.y = raw.y - pos_jump_offset_.y;
        this->odom_sub_flag = true;
    }

    // 위치 튐 보호 중인지 (offset 이 남아 있는지)
    bool is_position_jump_active()
    {
        return std::hypot(pos_jump_offset_.x, pos_jump_offset_.y) > pos_jump_active_;
    }

    double get_position_jump_offset()
    {
        return std::hypot(pos_jump_offset_.x, pos_jump_offset_.y);
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

        // 플래닝은 local_path 의 점마다 방향을 하나씩 보낸다. yaws[k] = k 번째 점의 방향.
        // (10/9 수정: 예전에는 2번 칸부터 두 칸씩 건너뛰어 읽어서 약 2배 먼 점의 방향을 썼다.
        //  그 '앞을 미리 보는' 효과는 이제 게인 파일의 stanley_yaw_lookahead 로 명시적으로 준다.)
        this->pl_local_path_yaw = msg->data;
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

        // 경로 선까지의 수직 거리 (+ = 차가 경로 오른쪽).
        // 10/9 수정: 예전에는 가장 가까운 '점'까지의 거리를 썼다. 점 간격(5 cm) 때문에 앞뒤 거리가 섞여
        // 0 근처에서 값이 튀었다. 가장 가까운 점 앞뒤 두 구간 중 차가 수직으로 내려앉는 구간을 쓴다.
        // 차가 경로 시작점보다 뒤에 있거나 끝점보다 앞에 있으면 (수직으로 내려앉을 구간이 없으면)
        // 예전처럼 점까지의 거리를 쓴다. (경로 시작점 앞에서 출발할 때 오차가 크게 보여야 적분을 비울 수 있다.)
        const int n = static_cast<int>(pl_local_path.size());
        double best_abs = 1e9;
        double signed_err = 0.0;
        bool found = false;
        for (int seg = std::max(0, closest_index - 1); seg <= std::min(closest_index, n - 2); seg++)
        {
            const Point &a = pl_local_path[seg];
            const Point &b = pl_local_path[seg + 1];
            double tx = b.x - a.x, ty = b.y - a.y;
            double len2 = tx * tx + ty * ty;
            if (len2 < 1e-12)
            {
                continue;
            }
            double u = ((pridict_pose.x - a.x) * tx + (pridict_pose.y - a.y) * ty) / len2;
            if (u < 0.0 || u > 1.0)
            {
                continue;
            }
            double cross = tx * (pridict_pose.y - a.y) - ty * (pridict_pose.x - a.x);
            double e = -cross / std::sqrt(len2);
            if (std::abs(e) < best_abs)
            {
                best_abs = std::abs(e);
                signed_err = e;
                found = true;
            }
        }
        if (found)
        {
            return signed_err;
        }

        // 수직으로 내려앉을 구간이 없을 때: 예전 방식 (가장 가까운 점까지의 거리, 좌우 부호)
        int seg = std::min(closest_index, n - 2);
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
        // path_yaw 개수가 경로 점 개수와 다를 때를 대비해 범위를 넘지 않게 한다.
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
        // i x dt 초 뒤의 위치와 헤딩을 예측한다 (차 기준 좌표, x 앞 / y 왼쪽).
        // 지금 진행 방향(beta)으로 출발해서 지금 회전 속도로 원호를 그린다고 본다.
        // 10/9 수정: 예전에는 헤딩을 모든 i 에서 (yaw + yaw_rate x dt) 로 같게 두었고 (x i 가 빠짐),
        //            위치도 회전 없이 직선으로만 예측했다.
        vector<Point> predict_pos;
        vector<float> predict_yaw;

        double slip = nomalize_angle(lo_beta - lo_yaw); // 차 방향 대비 진행 방향

        for (int i = 1; i <= n; i++)
        {
            double t = dt * i;
            double turn = this->lo_yaw_rate * t;    // t 초 동안 도는 각도
            double dist = this->speed * t;          // t 초 동안 가는 거리 (짧은 원호라 현 길이와 같다고 본다)
            double chord_dir = slip + 0.5 * turn;   // 원호의 시작점→끝점 방향

            Point tmp_odom;
            tmp_odom.x = dist * cos(chord_dir);
            tmp_odom.y = dist * sin(chord_dir);
            predict_pos.push_back(tmp_odom);

            predict_yaw.push_back(static_cast<float>(this->lo_yaw + turn));
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

    // yaw_lookahead_m_ 를 경로 점 개수로 바꾼다 (점 간격은 경로 앞 두 점에서 잰다).
    size_t lookahead_points()
    {
        if (this->yaw_lookahead_m_ <= 0.0 || pl_local_path.size() < 2)
        {
            return 0;
        }
        double spacing = std::hypot(pl_local_path[1].x - pl_local_path[0].x, pl_local_path[1].y - pl_local_path[0].y);
        if (spacing < 1e-3)
        {
            return 0;
        }
        return static_cast<size_t>(std::lround(this->yaw_lookahead_m_ / spacing));
    }

    double get_pd_path_yaw()
    {
        if (this->pl_local_path_yaw.empty())
        {
            return 404;
        }
        // 가장 가까운 점에서 yaw_lookahead_m_ 만큼 앞의 점 방향을 쓴다.
        size_t index = static_cast<size_t>(std::max(this->closest_index, 0)) + lookahead_points();
        index = std::min(index, this->pl_local_path_yaw.size() - 1);
        return this->pl_local_path_yaw[index];
    }

    // 차량 모드 (/ERP/serial_data[0], 1 = 자율)
    bool is_auto_mode()
    {
        return this->control_mode == 1.0F;
    }

    void set_yaw_lookahead(double lookahead_m)
    {
        this->yaw_lookahead_m_ = std::max(0.0, lookahead_m);
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