#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "control/callback_data_manage.hpp"
#include "control/PIDController.hpp"
#include "control/lat_control.hpp"
#include "control/lon_control.hpp"
#include "control/mission_param.hpp"
#include "control/vehicle_param.hpp"

#include <chrono>

using namespace std;

/*
제작 : 예원태
문의 : solnox99@koreatech.ac.kr

설명 :
ERP42 Pro 용 Control 코드의 main 코드이다. (2023년 창작차 대회 코드를 ERP42 Pro 에 맞게 고침)
print 부분을 제외한 모든 곳에서 SI단위를 준수한다.

받는 것
    /Local/utm, /Local/heading, /beta, /imu          로컬라이제이션
    /Planning/local_path, /Planning/path_yaw,
    /Planning/target_velocity, /Planning/mission     플래닝
    /ERP/serial_data                                  차량 상태 (브릿지)
        [mode, e_stop, gear, speed(m/s), steer(rad), brake, 0]

내보내는 것
    /Control/vehicle_cmd    차량 명령 (브릿지가 받아 CAN 으로 보낸다)
        [valid, e_stop, gear, speed(m/s), steer(rad), brake(%)]
        gear : 0 P / 1 D / 2 N / 3 R (ERP42 Pro 와 같은 값)
    /Control/tmp_plot_val   분석용 값 (아래 timer_callback 의 번호 참고)

특이사항 :
1. json 파일을 읽기 위해 "nlohmann" 모듈을 설치해 주어야 한다.
   <<  sudo apt install nlohmann-json3-dev  >>

2. 주의!!!
D, I 제어는 연산 주기에 매우 영향을 많이 받는다.
단순히 Hz를 높이려고 파라미터를 바꾸면 어떠한 일이 생길지 많이 고민해 보길 바람.
*/

namespace
{
const int CONTROL_PERIOD_MS = 10;   // 제어 주기 (100 Hz)
const int PRINT_EVERY_N_TICKS = 20; // 화면 출력 주기 (5 Hz)
const bool IS_PRINT = true;

const double TARGET_SPEED_MAX = 7.0; // 플래닝이 주는 목표 속도로 인정하는 상한 [m/s]
const double REVERSE_SPEED = 1.5;    // 후진 속도 [m/s]
const double FALLBACK_SPEED = 1.0;   // 목표 속도 값이 이상할 때 쓰는 속도 [m/s]

const int MISSION_NARROW = 998; // 협로 2바퀴째 (곡률에 따라 속도를 줄이는 미션)
} // namespace

class ERPControl : public rclcpp::Node
{
private:
    std::shared_ptr<CallbackClass> callback_data_ptr;
    CallbackClass *cb_data;

    std::shared_ptr<CombinedSteer> lat_control_ptr;
    CombinedSteer *lat_control;
    std::shared_ptr<ReversePurePursuit> R_lat_control_ptr;
    ReversePurePursuit *R_lat_control;

    std::shared_ptr<LonController> lon_control_ptr;
    LonController *lon_control;

    std::shared_ptr<ControlGainTuning> param_manage_ptr;
    ControlGainTuning *param_manage;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr lo_odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lo_curr_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr lo_imu__sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lo_beta_sub;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pl_loca_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr pl_miss_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pl_cont_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pl_pyaws_sub;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr er_data_sub;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vehicle_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tmp_data_pub;

    rclcpp::TimerBase::SharedPtr timer_;

    // 첫 계산 전까지는 정지 명령
    GasAndBrake acc_val{0.0, vehicle::BRAKE_STOP};

    long tick_count_ = 0;
    long not_ready_count_ = 0;

public:
    ERPControl()
        : Node("erp_control")
    {
        // <클레스들 인스턴스화>------------------------------------------------------
        callback_data_ptr = std::make_shared<CallbackClass>();
        cb_data = callback_data_ptr.get();

        lat_control_ptr = std::make_shared<CombinedSteer>(cb_data);
        lat_control = lat_control_ptr.get();
        R_lat_control_ptr = std::make_shared<ReversePurePursuit>();
        R_lat_control = R_lat_control_ptr.get();

        lon_control_ptr = std::make_shared<LonController>();
        lon_control = lon_control_ptr.get();

        param_manage_ptr = std::make_shared<ControlGainTuning>(cb_data, lat_control, R_lat_control, lon_control);
        param_manage = param_manage_ptr.get();
        // ------------------------------------------------------</클레스들 인스턴스화>
        // <SUBSCRIBER> -----------------------------------------------------------

        rclcpp::QoS UDP(rclcpp::KeepLast(1)); // Queue 사이즈 설정
        UDP.best_effort();                    // Best effort 설정

        // Local
        lo_odom_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/Local/utm", UDP, std::bind(&CallbackClass::lo_odom_cb, cb_data, std::placeholders::_1));
        lo_curr_sub = this->create_subscription<std_msgs::msg::Float64>("/Local/heading", UDP, std::bind(&CallbackClass::lo_yaw_cb, cb_data, std::placeholders::_1));
        lo_imu__sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu", UDP, std::bind(&CallbackClass::lo_imu_cb, cb_data, std::placeholders::_1));
        lo_beta_sub = this->create_subscription<std_msgs::msg::Float64>("/beta", UDP, std::bind(&CallbackClass::lo_beta_cb, cb_data, std::placeholders::_1));

        // Planning
        pl_loca_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Planning/local_path", UDP, std::bind(&CallbackClass::pl_local_path_cb, cb_data, std::placeholders::_1));
        pl_miss_sub = this->create_subscription<std_msgs::msg::Int16>("/Planning/mission", UDP, std::bind(&CallbackClass::pl_mission_num_cb, cb_data, std::placeholders::_1));
        pl_cont_sub = this->create_subscription<std_msgs::msg::Float64>("/Planning/target_velocity", UDP, std::bind(&CallbackClass::pl_control_switch_cb, cb_data, std::placeholders::_1));
        pl_pyaws_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Planning/path_yaw", UDP, std::bind(&CallbackClass::pl_local_path_yaws_cb, cb_data, std::placeholders::_1));

        // 차량 상태 (브릿지)
        er_data_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/ERP/serial_data", UDP, std::bind(&CallbackClass::co_ERP_data_cb, cb_data, std::placeholders::_1));

        // -----------------------------------------------------------</SUBSCRIBER>
        // <PUBLISHER> ------------------------------------------------------------

        vehicle_cmd_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Control/vehicle_cmd", UDP);
        tmp_data_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Control/tmp_plot_val", UDP);

        // ------------------------------------------------------------</PUBLISHER>

        timer_ = this->create_wall_timer(std::chrono::milliseconds(CONTROL_PERIOD_MS),
                                         std::bind(&ERPControl::timer_callback, this));
    }

    void timer_callback()
    {
        // 가독성을 위해 새로운 변수로 저장
        float curr_speed = cb_data->get_speed();
        float yaw = cb_data->get_yaw();
        Point odom = cb_data->get_odom();
        double lat_error = cb_data->calc_n_get_lat_error();
        double yaw_rate = cb_data->get_yawrate();
        double pd_path_yaw = cb_data->get_pd_path_yaw();
        double path_curature = 0.0, curv_1 = 0.0, curv_2 = 0.0, curv_3 = 0.0;

        // 위치나 경로가 준비되지 않았으면 명령을 내보내지 않는다.
        // (명령이 끊기면 브릿지가 차량을 세운다.)
        if (lat_error == 404 || pd_path_yaw == 404)
        {
            if (not_ready_count_++ % (1000 / CONTROL_PERIOD_MS) == 0)
            {
                const char *reason = cb_data->not_ready_reason();
                cout << "[대기] " << (reason != nullptr ? reason : "경로 또는 위치 값이 준비되지 않았습니다") << endl;
            }
            return;
        }
        not_ready_count_ = 0;

        // <계산> -----------------------------------------------------------
        int changed_param_num = param_manage->set_mission_param();
        int mission = cb_data->get_mission_num();

        // Control Switch (플래닝의 목표 속도. -1 : 후진 | 0 : 정지 | 0 초과 : 전진 속도 m/s)
        float cs = cb_data->get_control_switch();

        float steer = 0.0;
        int gear = vehicle::GEAR_D;

        // 횡방향 계산
        if (cs < 0.0)
        {
            gear = vehicle::GEAR_R;
            R_lat_control->set_R_PP_data(curr_speed, yaw, odom, cb_data->get_relative_path());
            steer = R_lat_control->calc_R_PP_steer();
        }
        else
        {
            if (mission == MISSION_NARROW && abs(lat_error) < 0.05)
            {
                lat_control->set_stanley_integral_val(0.0);
            }
            lat_control->set_stanly_data(curr_speed, pd_path_yaw, yaw, lat_error);
            lat_control->set_pp_data(curr_speed, yaw, odom, cb_data->get_relative_path());
            steer = lat_control->calc_combined_steer();
        }

        // 목표 속도 정하기
        if (cs == 0)
        {
            lon_control->set_lon_target_speed(0);
        }
        else if (0.0 < cs && cs <= TARGET_SPEED_MAX)
        {
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(cs);
        }
        else if (cs == -1)
        {
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(REVERSE_SPEED);
        }
        else
        {
            if (tick_count_ % (1000 / CONTROL_PERIOD_MS) == 0)
            {
                cout << "[경고] /Planning/target_velocity 값이 범위를 벗어났습니다 : " << cs << endl;
            }
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(FALLBACK_SPEED);
        }

        // 곡률에 따른 감속 (협로 2바퀴째 미션만)
        if (mission == MISSION_NARROW)
        {
            // 협로 첫 바퀴에는 곡률 데이터를 못믿는다.
            // 협로 두 번째 바퀴에서만 곡률에 따른 속도제어를 한다.
            curv_1 = abs(cb_data->calc_path_curvature(0.1, 1.5)); // 속도 제어시 제일 먼저 곡률을 계산해야함.
            curv_2 = abs(cb_data->calc_path_curvature(0.3, 1.5)); // S 커브와 같은 구간에서 불필요한 가속을 막기 위해
            curv_3 = abs(cb_data->calc_path_curvature(vehicle::STEER_DELAY, 1.5)); // 조향 지연 시간만큼 앞
            path_curature = max(curv_3, max(curv_1, curv_2));
            if (abs(path_curature) >= 0.1)
            {
                param_manage->target_speed_reducing_by_curature(path_curature);
            }
        }

        // 조향이 클 때의 감속
        if (cs != -1 && abs(path_curature) < 0.1)
        {
            param_manage->target_speed_reducing_by_steer(steer);
        }

        // 속도 명령과 브레이크 명령
        if (cs == 0)
        {
            acc_val.gas = 0.0;
            acc_val.brake = vehicle::BRAKE_STOP;
        }
        else if (cb_data->get_serial_sub_flag())
        {
            // 차량 상태를 새로 받았을 때만 다시 계산한다.
            acc_val = lon_control->calc_gas_n_brake();
            cb_data->set_down_serial_sub_flag();
        }

        if (mission == MISSION_NARROW)
        {
            acc_val.brake = clip(acc_val.brake, vehicle::BRAKE_NARROW_MIN, vehicle::BRAKE_NARROW_MAX);
        }

        // ---------------------------------------------------------- </계산>
        // <ros topic pub> -------------------------------------------------

        // 차량 명령
        std_msgs::msg::Float32MultiArray vehicle_cmd_msg;
        vehicle_cmd_msg.data = {
            1.0F,                              // 0 valid  (1 = 이 명령을 따를 것)
            0.0F,                              // 1 e_stop (1 = 비상 정지)
            static_cast<float>(gear),          // 2 gear   (0 P / 1 D / 2 N / 3 R)
            static_cast<float>(acc_val.gas),   // 3 speed  [m/s]
            steer,                             // 4 steer  [rad] (+ = 좌회전)
            static_cast<float>(acc_val.brake), // 5 brake  [%]
        };
        vehicle_cmd_pub->publish(vehicle_cmd_msg);

        // 분석용 토픽
        Point LDpoint = cb_data->AxisTrans_rel2abs(lat_control->get_target_point());

        std_msgs::msg::Float64MultiArray tmp_plot_val_msg;
        tmp_plot_val_msg.data.push_back(lat_error);                        // 0  경로와의 거리 [m]
        tmp_plot_val_msg.data.push_back(lon_control->get_target_speed());  // 1  목표 속도 [m/s]
        tmp_plot_val_msg.data.push_back(curv_1);                           // 2  곡률 (0.1 s 앞)
        tmp_plot_val_msg.data.push_back(curv_2);                           // 3  곡률 (0.3 s 앞)
        tmp_plot_val_msg.data.push_back(path_curature);                    // 4  감속에 쓴 곡률
        tmp_plot_val_msg.data.push_back(LDpoint.x);                        // 5  Pure Pursuit 목표점 x
        tmp_plot_val_msg.data.push_back(LDpoint.y);                        // 6  Pure Pursuit 목표점 y
        tmp_plot_val_msg.data.push_back(acc_val.gas);                      // 7  속도 명령 [m/s]
        tmp_plot_val_msg.data.push_back(acc_val.brake);                    // 8  브레이크 명령 [%]
        tmp_plot_val_msg.data.push_back(lat_control->get_stanly_steer());  // 9  Stanley 조향 [rad]
        vector<double> preview_steers = lat_control->get_preview_steers();
        for (size_t i = 0; i < preview_steers.size(); i++)
        {
            tmp_plot_val_msg.data.push_back(preview_steers[i]);            // 10 11 12  미리보기 조향 [rad]
        }
        tmp_plot_val_msg.data.push_back(lat_control->get_PP_steer());      // 13 Pure Pursuit 조향 [rad]

        vector<Point> predic_p = cb_data->get_stanley_predict_pos();
        for (size_t i = 0; i < predic_p.size(); i++)
        {
            Point tmp_p = cb_data->AxisTrans_rel2abs(predic_p[i]);
            tmp_plot_val_msg.data.push_back(tmp_p.x);                      // 14 16 18  미리보기 위치 x
            tmp_plot_val_msg.data.push_back(tmp_p.y);                      // 15 17 19  미리보기 위치 y
        }
        tmp_plot_val_msg.data.push_back(lat_control->get_stanley_integral_val()); // 20 Stanley 적분값
        tmp_plot_val_msg.data.push_back(curr_speed);                       // 21 차량 속도 [m/s]
        tmp_plot_val_msg.data.push_back(steer);                            // 22 조향 명령 [rad]
        tmp_data_pub->publish(tmp_plot_val_msg);

        // ------------------------------------------------ </ros topic pub>
        // <print> ---------------------------------------------------------

        if (IS_PRINT && tick_count_ % PRINT_EVERY_N_TICKS == 0)
        {
            cout << "==================" << endl;
            cout << "mission : " << mission << " (changed_param_num : " << changed_param_num << ")" << endl;
            cout << "yawrate : " << yaw_rate << endl;
            cout << "yaw : " << yaw << endl;
            cout << "lat_error : " << lat_error << endl;
            cout << "LD distance : " << lat_control->get_LA_distance() << endl;
            cout << "path_yaw_nearest(rad) : " << pd_path_yaw << endl;
            cout << "brake cmd : " << acc_val.brake << " %" << endl
                 << endl;

            cout << "speed cmd : " << acc_val.gas * 3.6 << "km/h (" << acc_val.gas << "m/s)" << endl;
            cout << "curr_speed : " << curr_speed * 3.6 << "km/h (" << curr_speed << "m/s)" << endl;
            cout << "target speed : " << lon_control->get_target_speed() * 3.6 << "km/h (" << lon_control->get_target_speed() << "m/s)" << endl;
            cout << "steer : " << steer * 180 / M_PI << " deg (" << steer << " rad)" << endl;
            cout << "PP_steer : " << lat_control->get_PP_steer() * 180 / M_PI << " deg (" << lat_control->get_PP_steer() << " rad)" << endl;
            cout << "stanly_steer: " << lat_control->get_stanly_steer() * 180 / M_PI << " deg (" << lat_control->get_stanly_steer() << " rad)" << endl;
        }
        tick_count_++;

        // ------------------------------------------------------- </print>
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ERPControl>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[오류] " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();

    return 0;
}
