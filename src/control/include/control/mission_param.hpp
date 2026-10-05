#ifndef GAIN_TUNING_HPP
#define GAIN_TUNING_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <climits>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "control/callback_data_manage.hpp"
#include "control/lat_control.hpp"
#include "control/lon_control.hpp"

using json = nlohmann::json;

// 게인 파일 이름 (확장자 제외). 설치된 control/config의 json을 읽는다.
// 현재 워크스페이스의 B_wynz 게인을 유지한다.
const std::string GAIN_FILE_NAME = "B_wynz";

/*
제작 : 예원태
문의 : solnox99@koreatech.ac.kr

설명 :
게인 튜닝을 json 파일로 하게 해 주는 코드이다.
normal 값을 먼저 적용한 뒤, 지금 미션 번호에 해당하는 값이 있으면 그 값으로 덮어쓴다.

json 파일을 읽기 위해 nlohmann 모듈이 필요하다.
   <<  sudo apt install nlohmann-json3-dev  >>

주의 :
미션별로 바꿔야 하는 값이 아니면 미션 칸에는 아무것도 적지 않는다.

    ex) 15번 미션에서 Pure Pursuit 만 쓰려면 바꿀 값만 적는다.

            "15": {
                "_":"터널 소형",
                "______---------------______": {
                    "PP_LD": [4.0, 11.0],
                    "combine_PP_ratio": 1.0
                }
            },

종방향(______lllllllllllllll______) 에서 읽는 값 (ERP42 Pro)
    "PD_brake_gain": [P, D]     목표보다 많이 빠를 때 거는 브레이크의 게인
    "enable_brake_error": 값    목표보다 (2 x 값) m/s 넘게 빠르면 브레이크를 건다
ERP42 Pro 는 속도를 차량이 스스로 맞추므로 가속용 게인(PD_gas_gain)과 gear 는 읽지 않는다.
*/

class ControlGainTuning
{
private:
    CallbackClass *cb_data_;
    CombinedSteer *lat_con_;
    ReversePurePursuit *R_lat_con_;
    LonController *long_con_;

    json file_data;
    json missions_data;

    const std::string LON_KEY = "______lllllllllllllll______";
    const std::string LAT_KEY = "______---------------______";

    float curvature_gain = 0.0;
    float pre_curv_speed_clip = 0.0;

    static std::string gain_file_path()
    {
        return ament_index_cpp::get_package_share_directory("control") +
            "/config/" + GAIN_FILE_NAME + ".json";
    }

public:
    ControlGainTuning(CallbackClass *cb_data, CombinedSteer *lat_con, ReversePurePursuit *R_lat_con, LonController *long_con)
    {
        this->cb_data_ = cb_data;
        this->lat_con_ = lat_con;
        this->R_lat_con_ = R_lat_con;
        this->long_con_ = long_con;

        read_json();
    }

    // 게인 파일을 읽는다. 읽지 못하면 이유를 알리고 컨트롤을 시작하지 않는다.
    void read_json()
    {
        std::string file_path = gain_file_path();

        std::ifstream input_file(file_path);
        if (!input_file.is_open())
        {
            throw std::runtime_error("게인 파일을 열 수 없습니다: " + file_path);
        }

        try
        {
            input_file >> this->file_data;
            this->missions_data = file_data.at("missions");
            this->curvature_gain = file_data.at("safety_factor").at("curvature_gain");
            // normal 에 필요한 값이 모두 있는지 시작할 때 한 번 확인한다.
            set_normal_param();
        }
        catch (const json::exception &e)
        {
            throw std::runtime_error("게인 파일 내용이 잘못되었습니다: " + file_path + " (" + e.what() + ")");
        }

        std::cout << "gain file : " << file_path << std::endl;
    }

    void set_normal_param()
    {
        const json &normal_data = file_data.at("normal");

        // longitudinal param
        const json &lon_param = normal_data.at(LON_KEY);
        long_con_->set_brake_PD_gain(lon_param.at("PD_brake_gain").at(0), lon_param.at("PD_brake_gain").at(1));
        long_con_->set_enable_brake_error(lon_param.at("enable_brake_error"));

        // lateral param
        const json &lat_param = normal_data.at(LAT_KEY);
        // stanly------------------------------------------------------------------------
        lat_con_->set_stanly_gain(lat_param.at("stanly_gain").at(0), lat_param.at("stanly_gain").at(1), lat_param.at("stanly_gain").at(2));
        lat_con_->set_heading_gain(lat_param.at("stanly_heading_gain"));
        lat_con_->set_anti_windup_max(lat_param.at("anti_windup_val"));

        // preview
        std::vector<float> preview_gain = lat_param.at("stanley_preview_gain");
        lat_con_->set_preview_param(lat_param.at("stanley_preview_dt"), preview_gain);
        lat_con_->set_preview_heading_error_gain(lat_param.at("stanley_preview_h_e_gain"));

        // pp------------------------------------------------------------------------------
        lat_con_->set_pp_LD_threshold(lat_param.at("PP_LD").at(0), lat_param.at("PP_LD").at(1));
        lat_con_->set_pp_gain(lat_param.at("PP_gain").at(0), lat_param.at("PP_gain").at(1));

        // reverse pp------------------------------------------------------------------------------
        R_lat_con_->set_R_PP_gain(lat_param.at("R_PP_gain").at(0), lat_param.at("R_PP_gain").at(1));
        R_lat_con_->set_R_pp_LD_threshold(lat_param.at("reverse_PP_LD").at(0), lat_param.at("reverse_PP_LD").at(1));

        // combine------------------------------------------------------------------------------
        lat_con_->set_combine_PP_ratio(lat_param.at("combine_PP_ratio"));
    }

    // normal 값을 적용한 뒤 지금 미션의 값으로 덮어쓴다. 반환: 미션 때문에 바뀐 항목 수
    int set_mission_param()
    {
        this->set_normal_param();

        int param_change_count = 0;
        std::string mission_key = std::to_string(cb_data_->get_mission_num());

        if (missions_data.find(mission_key) == missions_data.end())
        {
            return param_change_count;
        }
        const json &mission_data = missions_data.at(mission_key);

        if (mission_data.find(LON_KEY) != mission_data.end())
        {
            const json &lon_param = mission_data.at(LON_KEY);
            if (lon_param.find("PD_brake_gain") != lon_param.end())
            {
                param_change_count++;
                long_con_->set_brake_PD_gain(lon_param.at("PD_brake_gain").at(0), lon_param.at("PD_brake_gain").at(1));
            }
            if (lon_param.find("enable_brake_error") != lon_param.end())
            {
                param_change_count++;
                long_con_->set_enable_brake_error(lon_param.at("enable_brake_error"));
            }
        }

        if (mission_data.find(LAT_KEY) != mission_data.end())
        {
            const json &lat_param = mission_data.at(LAT_KEY);
            // stanly------------------------------------------------------------------------
            if (lat_param.find("stanly_gain") != lat_param.end())
            {
                param_change_count++;
                lat_con_->set_stanly_gain(lat_param.at("stanly_gain").at(0), lat_param.at("stanly_gain").at(1), lat_param.at("stanly_gain").at(2));
            }
            if (lat_param.find("anti_windup_val") != lat_param.end())
            {
                param_change_count++;
                lat_con_->set_anti_windup_max(lat_param.at("anti_windup_val"));
            }
            if (lat_param.find("stanly_heading_gain") != lat_param.end())
            {
                param_change_count++;
                lat_con_->set_heading_gain(lat_param.at("stanly_heading_gain"));
            }

            // preview (dt 와 gain 은 같이 적어야 한다)
            if (lat_param.find("stanley_preview_dt") != lat_param.end() ||
                lat_param.find("stanley_preview_gain") != lat_param.end())
            {
                try
                {
                    std::vector<float> preview_gain = lat_param.at("stanley_preview_gain");
                    lat_con_->set_preview_param(lat_param.at("stanley_preview_dt"), preview_gain);
                    param_change_count++;
                }
                catch (const json::exception &)
                {
                    std::cerr << "미션 " << mission_key << ": stanley_preview_dt 와 stanley_preview_gain 은 같이 적어야 합니다." << std::endl;
                    std::exit(EXIT_FAILURE);
                }
            }
            if (lat_param.find("stanley_preview_h_e_gain") != lat_param.end())
            {
                param_change_count++;
                lat_con_->set_preview_heading_error_gain(lat_param.at("stanley_preview_h_e_gain"));
            }

            // pp------------------------------------------------------------------------------
            if (lat_param.find("PP_LD") != lat_param.end())
            {
                param_change_count++;
                lat_con_->set_pp_LD_threshold(lat_param.at("PP_LD").at(0), lat_param.at("PP_LD").at(1));
            }
            if (lat_param.find("PP_gain") != lat_param.end())
            {
                param_change_count++;
                lat_con_->set_pp_gain(lat_param.at("PP_gain").at(0), lat_param.at("PP_gain").at(1));
            }
            // reverse pp------------------------------------------------------------------------------
            if (lat_param.find("R_PP_gain") != lat_param.end())
            {
                param_change_count++;
                R_lat_con_->set_R_PP_gain(lat_param.at("R_PP_gain").at(0), lat_param.at("R_PP_gain").at(1));
            }
            if (lat_param.find("reverse_PP_LD") != lat_param.end())
            {
                param_change_count++;
                R_lat_con_->set_R_pp_LD_threshold(lat_param.at("reverse_PP_LD").at(0), lat_param.at("reverse_PP_LD").at(1));
            }
            // combine------------------------------------------------------------------------------
            if (lat_param.find("combine_PP_ratio") != lat_param.end())
            {
                param_change_count++;
                lat_con_->set_combine_PP_ratio(lat_param.at("combine_PP_ratio"));
            }
        }

        return param_change_count;
    }

    void target_speed_reducing_by_curature(double curvature)
    {
        // 곡률로 급한 커브에서의 속도를 정하고, 미션 속도보다 낮으면 그 속도로 줄인다.
        float mission_speed = long_con_->get_target_speed();

        double curv = clip(curvature, 0.1, 1.5);
        float curvature_speed = sqrt(this->curvature_gain / curv);

        // mission speed로 클립
        float curv_speed_clip = clip(curvature_speed, 1.9F, mission_speed);

        double reducing_speed = low_pass_filter(curv_speed_clip, pre_curv_speed_clip, 0.9);
        pre_curv_speed_clip = reducing_speed;

        // 종방향 제어기 인스턴스에 바로 값을 넣어준다.
        long_con_->set_lon_target_speed(reducing_speed);
    }

    void target_speed_reducing_by_steer(float steer_cmd)
    {
        // 급한 조향을 하면서 가속을 하면 발산하기 쉽다.
        // 조향 명령과 실제 조향각 중 큰 쪽(크기 기준)으로 감속 비율을 정한다.
        float max_steer = std::max(std::abs(steer_cmd), std::abs(cb_data_->get_steer()));

        float mission_speed = long_con_->get_target_speed();
        double speed_gain = 1;

        if (max_steer > 26.0 * 3.141592 / 180.0)
        {
            speed_gain = 0.87;
        }
        else if (max_steer > 22.0 * 3.141592 / 180.0)
        {
            speed_gain = 0.91;
        }
        else if (max_steer > 17.0 * 3.141592 / 180.0)
        {
            speed_gain = 0.93;
        }
        else if (max_steer > 12.0 * 3.141592 / 180.0)
        {
            speed_gain = 0.95;
        }

        // 종방향 제어기 인스턴스에 바로 값을 넣어준다.
        long_con_->set_lon_target_speed(mission_speed * speed_gain);
    }
};

#endif // GAIN_TUNING_HPP
