#ifndef LON_CONTROLLER_HPP
#define LON_CONTROLLER_HPP

#include <cmath>
#include <memory>

#include "control/PIDController.hpp"
#include "control/vehicle_param.hpp"

/*
종방향 제어 (ERP42 Pro 용).

ERP42 Pro 는 받은 속도 명령을 차량이 스스로 맞춘다. 그래서
  - 속도 명령 : 목표 속도를 그대로 보낸다.
  - 브레이크  : 목표보다 (2 x enable_brake_error) 넘게 빠를 때만 PD 로 계산해 감속을 돕는다.

예전 ERP42 용 코드에 있던 것 중 뺀 것
  - 목표보다 큰 값을 보내 가속을 밀어붙이는 PD (gas PD)
  - 6 m/s 이상에서의 속도 보정 계수
브레이크 계산식과 게인은 예전과 같고, 눈금(0~200 -> %)과 계산 주기(0.05 s -> 0.02 s)만
ERP42 Pro 에 맞췄다 (vehicle_param.hpp).
*/

class LonController
{
private:
    std::shared_ptr<PIDController> brake_pid;

    float current_speed_ = 0.0;
    float target_speed_ = 0.0;
    float enable_brake_error = 0.6;

    // 브레이크 PD 의 미분 필터 계수.
    // 게인은 예전 ERP42 (상태 주기 0.05 s, 계수 0.3) 에서 맞춘 것이다. 주기가 0.02 s 로 바뀌어도
    // 미분값과 필터가 시간으로 볼 때 예전과 같게 움직이도록 계수를 주기에 맞춰 환산한다 (약 0.13).
    const float d_filter_alpha = 1.0 - std::pow(1.0 - 0.3, vehicle::FEEDBACK_PERIOD / 0.05);

public:
    LonController()
    {
        brake_pid = std::make_shared<PIDController>(-1.0, 1.0);
    }

    void set_lon_data(float current_speed)
    {
        this->current_speed_ = current_speed;
    }

    void set_lon_target_speed(float target_speed)
    {
        this->target_speed_ = target_speed;
    }

    float get_target_speed()
    {
        return this->target_speed_;
    }

    void set_enable_brake_error(float enable_error)
    {
        this->enable_brake_error = enable_error;
    }

    void set_brake_PD_gain(double br_kp, double br_kd)
    {
        // 목표 속도가 1.7 ~ 2.5 m/s 일 때 P 게인을 조금 키운다 (예전 코드의 실제 동작과 같음).
        if (this->target_speed_ < 2.5 && this->target_speed_ > 1.7)
        {
            br_kp += this->target_speed_ * 0.004;
        }
        brake_pid->set_PID_gain(br_kp, br_kd);
    }

    // 반환: gas = 속도 명령 [m/s], brake = 브레이크 명령 [%]
    GasAndBrake calc_gas_n_brake()
    {
        float error = target_speed_ - current_speed_;

        GasAndBrake return_val;
        return_val.gas = clip(static_cast<double>(target_speed_), 0.0, vehicle::MAX_SPEED);

        if (-2 * this->enable_brake_error < error)
        {
            // 목표보다 느리거나 조금 빠름 -> 브레이크 없음
            return_val.brake = 0.0;
        }
        else
        {
            // 목표보다 많이 빠름 -> 브레이크로 감속을 돕는다
            double brake_val = brake_pid->compute(target_speed_, current_speed_, d_filter_alpha, vehicle::FEEDBACK_PERIOD);
            return_val.brake = vehicle::BRAKE_BASE - brake_val * vehicle::BRAKE_SCALE;
        }
        return_val.brake = clip(return_val.brake, 0.0, vehicle::BRAKE_MAX);

        return return_val;
    }
};

#endif // LON_CONTROLLER_HPP
