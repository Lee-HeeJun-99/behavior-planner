#ifndef GAIN_TUNING_HPP
#define GAIN_TUNING_HPP

#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>
#include <libgen.h>
#include <unistd.h>
#include <stdexcept> // for std::runtime_error
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "callback_data_manage.hpp"
#include "lat_control.hpp"
#include "lon_control.hpp"

using json = nlohmann::json;

// std::string file_name = "A";           // 케이법
std::string file_name = "B_wynz";   // 코러스
// std::string file_name = "B";
// std::string file_name = "P";

/*
제작 : 예원태
문의 : solnox99@koreatech.ac.kr

설명 :
2023년 창작차 대회에서 사용한 Control 코드이다.
Gain 튜닝을 원활히 하기 위해 Json 파일을 사용하게 해 주는 코드이다.
normal param을 먼저 설정한 후에
mission에 따른 param을 덮어쓰는 형식으로 동작한다.

특이사항 :
1. package.xml에 dependency를 걸지 않았지만
json 파일을 읽기 위해 "nlohmann" 모듈을 설치해 주어야 한다.
   <<  sudo apt install nlohmann-json3-dev  >>

주의 :
1. 미션별로 설정해 줘야하는 파라미터가 아니면
    제발제발 Json 파일에 아무것도 적지마라.
    가독성 떨어지고 헷갈리고 코드 꼬인다.
    제발 적지마라.

    ex) 15번 미션에서는 only PP 만을 써야하기 때문에
        "combine_PP_ratio"를 1.0으로 작성한다.
        아래를 보면 총 3개의 param을 바꿨다.

            "15": {
                "_":"터널 소형",
                "______lllllllllllllll______": {
                    "target_speed": 2.5
                },
                "______---------------______": {
                    "PP_LD": [4.0, 11.0],
                    "combine_PP_ratio": 1.0
                }
            },

        11번 미션에서는 target speed만을 기입해 속도만 줄인다.
        나머지 모든 param은 normal로 사용한다.

            "11": {
                "_":"소형",
                "______lllllllllllllll______": {
                    "target_speed": 3.0
                }
            },



*/

class ControlGainTuning
{
private:
    CallbackClass *cb_data_;
    CombinedSteer *lat_con_;
    ReversePurePursuit *R_lat_con_;
    LonController *long_con_;

    float gear = 0.0; // 0: 전진, 1: 중립, 2: 후진

    json file_data;
    json missions_data;

    float curvature_gain = 0.0;
    float pre_curv_speed_clip = 0.0;

public:
    ControlGainTuning(CallbackClass *cb_data, CombinedSteer *lat_con, ReversePurePursuit *R_lat_con, LonController *long_con)
    {
        this->cb_data_ = cb_data;
        this->lat_con_ = lat_con;
        this->R_lat_con_ = R_lat_con;
        this->long_con_ = long_con;

        read_json();
        // TODO: pp에 min, max LD 값 Json으로 읽어오기.
    }

    int read_json()
    {
        const std::string file_path =
            ament_index_cpp::get_package_share_directory("control") + "/config/" + file_name + ".json";

        // JSON 파일 읽기
        std::ifstream input_file(file_path);

        if (!input_file.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ControlGainTuning"), "Error opening JSON file: %s", file_path.c_str());
            return 1;
        }

        // JSON 파싱
        json data;
        try
        {
            input_file >> data;
        }
        catch (const json::parse_error &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ControlGainTuning"), "JSON parse error: %s", e.what());
            input_file.close();
            return 1;
        }

        this->file_data = data;
        this->missions_data = file_data["missions"];

        // safety_factor를 읽어온다.
        this->curvature_gain = this->file_data["safety_factor"]["curvature_gain"];

        return 0;
    }

    void set_normal_param()
    {
        json normal_data = file_data["normal"];

        // longitudinal param
        json lon_param = normal_data["______lllllllllllllll______"];
        long_con_->set_lon_PD_gain(lon_param["PD_gas_gain"][0], lon_param["PD_gas_gain"][1],
                                   lon_param["PD_brake_gain"][0], lon_param["PD_brake_gain"][1]);
        // long_con_->set_lon_target_speed(lon_param["target_speed"]);

        long_con_->set_enable_brake_error(lon_param["enable_brake_error"]);
        this->gear = lon_param["gear"];

        // latarl param
        json lat_param = normal_data["______---------------______"];
        // stanly------------------------------------------------------------------------
        lat_con_->set_stanly_gain(lat_param["stanly_gain"][0], lat_param["stanly_gain"][1], lat_param["stanly_gain"][2]);
        lat_con_->set_heading_gain(lat_param["stanly_heading_gain"]);
        lat_con_->set_anti_windup_max(lat_param["anti_windup_val"]);

        // preview
        std::vector<float> preview_gain = lat_param["stanley_preview_gain"];
        lat_con_->set_preview_param(lat_param["stanley_preview_dt"], preview_gain);
        lat_con_->set_preview_heading_error_gain(lat_param["stanley_preview_h_e_gain"]);

        // pp------------------------------------------------------------------------------
        lat_con_->set_pp_LD_threshold(lat_param["PP_LD"][0], lat_param["PP_LD"][1]);
        lat_con_->set_pp_gain(lat_param["PP_gain"][0], lat_param["PP_gain"][1]);

        // reverse pp------------------------------------------------------------------------------
        R_lat_con_->set_R_PP_gain(lat_param["R_PP_gain"][0], lat_param["R_PP_gain"][1]);
        R_lat_con_->set_R_pp_LD_threshold(lat_param["reverse_PP_LD"][0], lat_param["reverse_PP_LD"][1]);

        // combine------------------------------------------------------------------------------
        lat_con_->set_combine_PP_ratio(lat_param["combine_PP_ratio"]);
    }

    int set_mission_param()
    {
        // 미션에 따라 달라지는 gain
        // nomalGain의 값을 override 하도록 함.

        // this->print_erp_name();
        this->set_normal_param();

        int param_change_count = 0;
        int mission = cb_data_->get_mission_num();

        std::string mission_key = std::to_string(mission);

        // json 파일에서 파라미터가 있는지 확인하고 있을 경우에만 업데이트 한다.
        if (missions_data.find(mission_key) != missions_data.end())
        {
            // 미션별 파라미터 적용
            if (missions_data[mission_key].find("______lllllllllllllll______") != missions_data[mission_key].end())
            {
                json lon_param = missions_data[mission_key]["______lllllllllllllll______"];
                // if (lon_param.find("target_speed") != lon_param.end())
                // {
                //     param_change_count++;
                //     long_con_->set_lon_target_speed(lon_param["target_speed"]);
                // }
                if (lon_param.find("PD_gas_gain") != lon_param.end() ||
                    lon_param.find("PD_brake_gain") != lon_param.end())
                {
                    try
                    {
                        param_change_count++;
                        long_con_->set_lon_PD_gain(lon_param["PD_gas_gain"][0], lon_param["PD_gas_gain"][1],
                                                   lon_param["PD_brake_gain"][0], lon_param["PD_brake_gain"][1]);
                    }
                    catch (...)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("ControlGainTuning"), "PD lon gain은 gas와 brake를 같이 작성해야함.");
                        std::exit(EXIT_FAILURE); // 프로그램 종료
                    }
                }
                if (lon_param.find("enable_brake_error") != lon_param.end())
                {
                    param_change_count++;
                    long_con_->set_enable_brake_error(lon_param["enable_brake_error"]);
                }
            }

            if (missions_data[mission_key].find("______---------------______") != missions_data[mission_key].end())
            {
                json lat_param = missions_data[mission_key]["______---------------______"];
                // stanly------------------------------------------------------------------------
                if (lat_param.find("stanly_gain") != lat_param.end())
                {
                    param_change_count++;
                    lat_con_->set_stanly_gain(lat_param["stanly_gain"][0], lat_param["stanly_gain"][1], lat_param["stanly_gain"][2]);
                }
                if (lat_param.find("anti_windup_val") != lat_param.end())
                {
                    param_change_count++;
                    lat_con_->set_anti_windup_max(lat_param["anti_windup_val"]);
                }
                if (lat_param.find("stanly_heading_gain") != lat_param.end())
                {
                    param_change_count++;
                    lat_con_->set_heading_gain(lat_param["stanly_heading_gain"]);
                }

                // preview
                if (lat_param.find("stanley_preview_dt") != lat_param.end() ||
                    lat_param.find("stanley_preview_gain") != lat_param.end())
                {
                    try
                    {
                        std::vector<float> preview_gain = lat_param["stanley_preview_gain"];
                        lat_con_->set_preview_param(lat_param["stanley_preview_dt"], preview_gain);
                        param_change_count++;
                    }
                    catch (...)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("ControlGainTuning"), "preview_param은 dt와 gain을 같이 작성해야함.");
                        std::exit(EXIT_FAILURE); // 프로그램 종료
                    }
                }
                if (lat_param.find("stanley_preview_h_e_gain") != lat_param.end())
                {
                    param_change_count++;
                    lat_con_->set_preview_heading_error_gain(lat_param["stanley_preview_h_e_gain"]);
                }

                // pp------------------------------------------------------------------------------
                if (lat_param.find("PP_LD") != lat_param.end())
                {
                    param_change_count++;
                    lat_con_->set_pp_LD_threshold(lat_param["PP_LD"][0], lat_param["PP_LD"][1]);
                }
                if (lat_param.find("PP_gain") != lat_param.end())
                {
                    param_change_count++;
                    lat_con_->set_pp_gain(lat_param["PP_gain"][0], lat_param["PP_gain"][1]);
                }
                // reverse pp------------------------------------------------------------------------------
                if (lat_param.find("R_PP_gain") != lat_param.end())
                {
                    param_change_count++;
                    R_lat_con_->set_R_PP_gain(lat_param["R_PP_gain"][0], lat_param["R_PP_gain"][1]);
                }
                if (lat_param.find("reverse_PP_LD") != lat_param.end())
                {
                    param_change_count++;
                    R_lat_con_->set_R_pp_LD_threshold(lat_param["reverse_PP_LD"][0], lat_param["reverse_PP_LD"][1]);
                }
                // combine------------------------------------------------------------------------------
                if (lat_param.find("combine_PP_ratio") != lat_param.end())
                {
                    param_change_count++;
                    lat_con_->set_combine_PP_ratio(lat_param["combine_PP_ratio"]);
                }
            }
        }

        return param_change_count;
    }

    void target_speed_reducing_by_curature(double curvature)
    {
        // curvature 값으로 급한 커브에서 속도를 결정하여,
        // mission speed보다 클 경우 속도를 줄여준다. (override)
        float curvature_speed = 0.0f;
        float mission_speed = long_con_->get_target_speed();

        double curv = clip(curvature, 0.1, 1.5);
        curvature_speed = std::sqrt(this->curvature_gain / curv);

        // mission speed로 클립
        float curv_speed_clip = clip(curvature_speed, 1.9F, mission_speed);

        double reducing_speed = low_pass_filter(curv_speed_clip, pre_curv_speed_clip, 0.9);
        pre_curv_speed_clip = reducing_speed;

        // 종방향 제어기 인스턴스에 바로 값을 넣어준다.
        long_con_->set_lon_target_speed(reducing_speed);
    }

    void target_speed_reducing_by_steer(float tmp_steer)
    {
        // 급한 조향을 하면서 가속을 하면 발산하기 쉽다.
        // 주로 곡률에 따른 속도 이후에 사용해야 한다.
        float max_steer = std::abs(tmp_steer);

        float erp_steer = std::abs(cb_data_->get_steer());

        if (erp_steer > tmp_steer)
        {
            max_steer = erp_steer;
        }

        float mission_speed = long_con_->get_target_speed();
        double speed_gain = 1;

        if (std::abs(max_steer) > 26.0 * 3.141592 / 180.0)
        {
            speed_gain = 0.92;
        }
        else if (std::abs(max_steer) > 22.0 * 3.141592 / 180.0)
        {
            speed_gain = 0.93;
        }
        else if (std::abs(max_steer) > 17.0 * 3.141592 / 180.0)
        {
            speed_gain = 0.94;
        }
        else if (std::abs(max_steer) > 12.0 * 3.141592 / 180.0)
        {
            speed_gain = 0.95;
        }

        // 종방향 제어기 인스턴스에 바로 값을 넣어준다.
        long_con_->set_lon_target_speed(mission_speed * speed_gain);
    }

    void print_erp_name()
    {
        // 기존 cout 연속 출력 대신 INFO 로그로 동일 정보 출력
        std::string repeated;
        repeated.reserve(file_name.size() * 20);
        for (int i = 0; i < 20; i++) repeated += file_name;
        RCLCPP_INFO(rclcpp::get_logger("ControlGainTuning"), "\n\n\nname : %s ERP!!!!!!!!", repeated.c_str());
    }

    float get_gear()
    {
        return this->gear;
    }
};

#endif // GAIN_TUNING_HPP
