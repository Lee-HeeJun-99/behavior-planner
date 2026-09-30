#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "control/callback_data_manage.hpp"
#include "control/PIDController.hpp"
#include "control/lat_control.hpp"
#include "control/lon_control.hpp"
#include "control/mission_param.hpp"

using namespace std;

bool IS_PRINT = true;

/*
제작 : 예원태
문의 : solnox99@koreatech.ac.kr

설명 :
2023년 창작차 대회에서 사용한 Control 코드의 main 코드이다.
print 부분을 제외한 모든 곳에서 SI단위를 준수한다.

제발제발 SI 단위써라. 제발.


특이사항 :
1. package.xml에 dependency를 걸지 않았지만
json 파일을 읽기 위해 "nlohmann" 모듈을 설치해 주어야 한다.
   <<  sudo apt install nlohmann-json3-dev  >>

2. 주의!!!
D, I 제어는 연산 주기에 매우 영향을 많이 받는다.
단순히 Hz를 높이려고 파라미터를 바꾸면 어떠한 일이 생길지 많이 고민해 보길 바람.

*/

float wheel_adaptation(float Yawrate_rps, float delta_offset_pre)
{
    float delta_offset = delta_offset_pre;

    if (Yawrate_rps > 0.01)
    {                                             // need tune (0.01rad)
        delta_offset = delta_offset_pre - 0.0001; // need tune (plus -0.0001rad)
    }
    else if (Yawrate_rps < -0.01)
    {                                             // need tune (-0.01rad)
        delta_offset = delta_offset_pre + 0.0001; // need tune (plus 0.0001rad)
    }
    return delta_offset;
}

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
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pl_pyaw_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pl_pyaws_sub;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr er_data_sub;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ERP_data_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tmp_data_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Rel_path_pub;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_data_pub;

    rclcpp::TimerBase::SharedPtr timer_;

    float pre_offset_delta = 0;
    GasAndBrake acc_val;

public:
    ERPControl()
        : Node("erp_control")
    // <ROS 노드 선언>---------------------------------------------------------
    // ---------------------------------------------------------</ROS 노드 선언>
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

        // UDP = rclcpp::QoS(rclcpp::SensorDataQoS(), rclcpp::KeepLast(1));
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
        // pl_pyaw_sub = this->create_subscription<std_msgs::msg::Float64>("/Planning/path_yaw", 1, std::bind(&CallbackClass::pl_path_yaw_cb, cb_data, std::placeholders::_1));
        pl_pyaws_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Planning/path_yaw", UDP, std::bind(&CallbackClass::pl_local_path_yaws_cb, cb_data, std::placeholders::_1));

        // ERP Serial Data
        er_data_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/ERP/serial_data", UDP, std::bind(&CallbackClass::co_ERP_data_cb, cb_data, std::placeholders::_1));

        // -----------------------------------------------------------</SUBSCRIBER>
        // <PUBLISHER> ------------------------------------------------------------s

        ERP_data_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Control/serial_data", UDP);
        tmp_data_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Control/tmp_plot_val", UDP);
        Rel_path_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Control/rel_path", UDP);
        control_data_pub = this->create_publisher<std_msgs::msg::Float64>("/Control/center_speed", UDP);

        // ------------------------------------------------------------</PUBLISHER>

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
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

        // lat_error가 404면 뭔가 잘못됨.
        if (lat_error == 404 || pd_path_yaw == 404)
        {
            // // pub E-STOP
            // auto ERP_data_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
            // ERP_data_msg->data = {1, 1, 0, 0, 0, 0, 0};
            // ERP_data_pub->publish(*ERP_data_msg);
            RCLCPP_WARN(this->get_logger(), "404!!!");
            return;
        }

        // <계산> -----------------------------------------------------------
        int changed_param_num = param_manage->set_mission_param();

        // Control Switch
        float cs = cb_data->get_control_switch();

        float steer = 0.0;
        int gear_state = 0;

        // 횡방향 계산
        if (cs < 0.0)
        {
            gear_state = 2;
            R_lat_control->set_R_PP_data(curr_speed, yaw, odom, cb_data->get_relative_path());
            steer = R_lat_control->calc_R_PP_steer();
        }
        else
        {
            if(cb_data->get_mission_num() == 998 && abs(lat_error) < 0.05)
            {
                lat_control->set_stanley_integral_val(0.0);
            }          
            lat_control->set_stanly_data(curr_speed, pd_path_yaw, yaw, lat_error);
            lat_control->set_pp_data(curr_speed, yaw, odom, cb_data->get_relative_path());
            steer = lat_control->calc_combined_steer();
        }

        if (cs == 0)
        {
            acc_val.gas = 0.0;
            acc_val.brake = 180;
            lon_control->set_lon_target_speed(0);
        }
        else if (0.0 < cs && cs <= 7.0)
        {
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(cs);
        }
        else if (cs == -1) // 후진시 종방향 제어 1007 평행주차 종방향 제어 속도 넣기
        {
            if (cb_data->get_mission_num() == 15) // 평행주차 후진 속도 1.5
            {
                lon_control->set_lon_data(curr_speed);
                lon_control->set_lon_target_speed(1.5);
            }
            else if (cb_data->get_mission_num() == 21) // 사선주차 후진 속도 1.0
            {
                lon_control->set_lon_data(curr_speed);
                lon_control->set_lon_target_speed(1.5); 
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "pl_control_switch error");
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(1.0);
        }

        // 종방향 계산
        if (cb_data->get_mission_num() == 998)
        {
            // 협로 첫 바퀴에는 곡률 데이터를 못믿는다.
            // 협로 두 번째 바퀴에서만 곡률에 따른 속도제어를 한다.
            curv_1 = abs(cb_data->calc_path_curvature(0.1, 2.0)); // 속도 제어시 제일 먼저 곡률을 계산해야함.
            curv_2 = abs(cb_data->calc_path_curvature(0.3, 2.0)); // S 커브와 같은 구간에서 불필요한 가속을 막기 위해
            curv_3 = abs(cb_data->calc_path_curvature(0.6, 2.0));
            path_curature = max({curv_3, curv_1, curv_2});
            // path_curature = max(curv_1,curv_2);

            param_manage->target_speed_reducing_by_curature(path_curature);
        }

        // if (cs != -1)
        // {
        //     param_manage->target_speed_reducing_by_steer(steer);
        // }

        if (cb_data->get_serial_sub_flag() && cs != 0) 
        {
            acc_val = lon_control->calc_gas_n_brake();
            cb_data->set_down_serial_sub_flag();
        }

        if (cb_data->get_mission_num() == 998)
        {
            acc_val.brake = clip(acc_val.brake, 20.0, 120.0);
        }

        // ---------------------------------------------------------- </계산>
        // <ros topic pub> -------------------------------------------------

        Point LDpoint;
        LDpoint = lat_control->get_target_point();
        LDpoint = cb_data->AxisTrans_rel2abs(LDpoint);

        // 제어 값 pub
        auto ERP_data_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();

        ERP_data_msg->data.push_back(1.0);           // control_mode
        ERP_data_msg->data.push_back(0.0);           // e_stop
        ERP_data_msg->data.push_back(gear_state);    // gear
        ERP_data_msg->data.push_back(acc_val.gas); // speed
        ERP_data_msg->data.push_back(steer);         // steer
        ERP_data_msg->data.push_back(acc_val.brake); // brake
        ERP_data_msg->data.push_back(0.0);           // enc
        ERP_data_msg->data.push_back(lon_control->get_lon_target_speed());           // enc


        ERP_data_pub->publish(*ERP_data_msg);

        //center_speed pub
        auto center_speed_msg = std::make_shared<std_msgs::msg::Float64>();
        center_speed_msg->data = cb_data->get_speed();
        control_data_pub->publish(*center_speed_msg);
        


        // 디버그용 토픽
        auto tmp_plot_val_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        // tmp_plot_val_msg->data.push_back(R_lat_control->get_reverse_yaw()); // 0
        // tmp_plot_val_msg->data.push_back(lat_control->get_LA_distance());   // 1
        tmp_plot_val_msg->data.push_back(lat_error);                       // 0
        tmp_plot_val_msg->data.push_back(lon_control->get_target_speed()); // 1
        tmp_plot_val_msg->data.push_back(curv_1);                          // 2
        tmp_plot_val_msg->data.push_back(curv_2);                          // 3
        tmp_plot_val_msg->data.push_back(path_curature);                   // 4
        tmp_plot_val_msg->data.push_back(LDpoint.x);                       // 5
        tmp_plot_val_msg->data.push_back(LDpoint.y);                       // 6

        tmp_plot_val_msg->data.push_back(lat_control->get_stanly_steer()); //7
        vector<double> preview_steers = lat_control->get_preview_steers();
        for (size_t i = 0; i < preview_steers.size(); i++)
        {
            tmp_plot_val_msg->data.push_back(preview_steers[i]); // 8 9 10
        }
        tmp_plot_val_msg->data.push_back(lat_control->get_PP_steer()); // 11

        vector<Point> predic_p = cb_data->get_stanley_predict_pos();
        for (size_t i = 0; i < predic_p.size(); i++)
        {
            Point tmp_p = cb_data->AxisTrans_rel2abs(predic_p[i]);
            tmp_plot_val_msg->data.push_back(tmp_p.x); //
            tmp_plot_val_msg->data.push_back(tmp_p.y); //
        }
        tmp_plot_val_msg->data.push_back(lat_control->get_stanley_integral_val());
        tmp_plot_val_msg->data.push_back(cb_data->get_speed());

        tmp_data_pub->publish(*tmp_plot_val_msg);

        // 상대경로를 plot을 위해 pub 하는 구문
        if (false)
        {
            vector<Point> Rel_path = cb_data->get_relative_path();

            auto Rel_path_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();

            for (size_t i = 1; i < Rel_path.size(); i++)
            {
                Rel_path_msg->data.push_back(Rel_path[i].x);
                Rel_path_msg->data.push_back(Rel_path[i].y);
            }

            Rel_path_pub->publish(*Rel_path_msg);
        }

        // ------------------------------------------------ </ros topic pub>
        // <print> ---------------------------------------------------------

        if (IS_PRINT)
        {
            RCLCPP_INFO(this->get_logger(), "==================");
            RCLCPP_INFO(this->get_logger(), "changed_param_num : %d", changed_param_num);
            RCLCPP_INFO(this->get_logger(), "gas : %.3f", acc_val.gas);
            RCLCPP_INFO(this->get_logger(), "brake : %.3f", acc_val.brake);
            RCLCPP_INFO(this->get_logger(), "steer : %.6f", steer);

            RCLCPP_INFO(this->get_logger(), "yawrate : %.6f", yaw_rate);
            RCLCPP_INFO(this->get_logger(), "PP_steer : %.3f deg (%.6f rad)", lat_control->get_PP_steer() * 180 / M_PI, lat_control->get_PP_steer());
            RCLCPP_INFO(this->get_logger(), "stanly_steer: %.3f deg (%.6f rad)", lat_control->get_stanly_steer() * 180 / M_PI, lat_control->get_stanly_steer());
            RCLCPP_INFO(this->get_logger(), "curr_speed : %.3f km/h (%.6f m/s)", curr_speed * 3.6, curr_speed);
            RCLCPP_INFO(this->get_logger(), "yaw : %.6f", yaw);
            RCLCPP_INFO(this->get_logger(), "lat_error : %.6f", lat_error);
            RCLCPP_INFO(this->get_logger(), "LD distance : %.6f", lat_control->get_LA_distance());
            RCLCPP_INFO(this->get_logger(), "target speed : %.3f km/h (%.6f m/s)", lon_control->get_target_speed() * 3.6, lon_control->get_target_speed());
            RCLCPP_INFO(this->get_logger(), "path_yaw_nearest(rad) : %.6f", pd_path_yaw);
        }

        // ------------------------------------------------------- </print>
        return;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ERPControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}