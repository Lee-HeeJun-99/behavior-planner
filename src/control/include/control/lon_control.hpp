#ifndef LON_CONTROLLER_HPP
#define LON_CONTROLLER_HPP

#include <iostream>

#include "control/PIDController.hpp"

/*
제작 : 예원태
문의 : solnox99@koreatech.ac.kr

설명 :
2023년 창작차 대회에서 사용한 Control 코드이다.
종방향 제어를 위한 PID 제어기가 정의되어 있다.

*/

class LonController
{
private:
    shared_ptr<PIDController> gas_pid;
    shared_ptr<PIDController> brake_pid;

    float current_speed_;
    float target_speed_;

    float gas_scale_;   // gas : 0 ~ 6.9444 m/s
    float brake_scale_; // brake : 0.f ~ 10

    float pre_calc_speed = 0.0;
    float pre_calc_brake = 0.0;
    float pre_target_speed = 0.0;
    float enable_brake_error = 0.6;

public:
    LonController()
        : gas_scale_(6.9), brake_scale_(200)
    {
        gas_pid = std::make_shared<PIDController>(-1.0, 1.0);
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

    double get_lon_target_speed()
    {
        return this->target_speed_;
    }

    void set_enable_brake_error(float enable_error)
    {
        this->enable_brake_error = enable_error;
    }

    void set_lon_PD_gain(double ga_kp, double ga_kd, double br_kp, double br_kd)
    {
        // i gain은 편의상 0.0으로 고정.
        gas_pid->set_PID_gain(ga_kp, ga_kd);
        brake_pid->set_PID_gain(br_kp, br_kd);

        // if (this->target_speed_ < 2.5) // 저속일때 감속을 더 빠르게 하고 목표속도에 빨리 도달하게 하기 위한 부분입니다.
        // {
        //     brake_pid->set_PID_gain(br_kp + 0.2, br_kd);
        // }
        if (this->pre_target_speed != this->target_speed_ && this->target_speed_ < 2.5 && this->target_speed_ > 1.7)
        {
            float speed_gap = abs(this->pre_target_speed - this->target_speed_);
            brake_pid->set_PID_gain(br_kp + speed_gap * 0.004, br_kd);
        }
    }

    GasAndBrake calc_gas_n_brake()
    {
        // float enable_error = 0.2;
        float error = target_speed_ - current_speed_;
        GasAndBrake return_val;
        double gas_val = gas_pid->compute(target_speed_, current_speed_, 0.1);

        if (this->target_speed_ < 2.5)
        {
            this->enable_brake_error = 0.6 * target_speed_ * 0.35;
        }
        else
        {
            this->enable_brake_error = this->enable_brake_error;
        }

        if (-this->enable_brake_error < error)
        {
            if(gas_val<0){      // 가속 상황에서 순간적으로 gas_val 음수 방지
                gas_val = 0;
            }
            return_val.gas = target_speed_ + gas_val * gas_scale_;
            return_val.brake = 0;
            // std::cout << "11111111111111111" << std::endl;
        }
        else if (-2 * this->enable_brake_error < error && error <= -this->enable_brake_error)
        {
            return_val.gas = target_speed_;
            return_val.brake = 0;
            // std::cout << "22222222222222222222222" << std::endl;
        }
        else
        {
            double brake_val = brake_pid->compute(target_speed_, current_speed_, 0.3);
            return_val.gas = 0 * gas_scale_;
            return_val.brake = 30 - brake_val * brake_scale_;
        }

        return_val.gas = clip(return_val.gas, 0.0, 6.9);

        if (current_speed_ >= 6.0)
        {
            if (std::abs(current_speed_ - target_speed_) < target_speed_ * 0.0144 * target_speed_ && error > 0)
            {
                return_val.gas = clip(return_val.gas, 0.0, static_cast<double>(target_speed_ * (1 - 0.0144 * target_speed_)));
            }
            else if (error <= 0)
            {
                return_val.gas = clip(return_val.gas, 0.0, static_cast<double>(target_speed_ * (1 - 0.0144 * target_speed_) + error));
            }
        }

        return_val.gas = low_pass_filter(return_val.gas, this->pre_calc_speed, 0.6);
        // if (this->target_speed_ < 2.5)
        //     return_val.brake = low_pass_filter(return_val.brake, this->pre_calc_brake, 0.9);
            
        if (this->target_speed_ < 1.7)
        {
            return_val.brake = clip(return_val.brake, 20.0, 199.0);
        }
        else
        {
            return_val.brake = clip(return_val.brake, 35.0, 199.0);
        }
        // return_val.brake = clip(return_val.brake, 35.0, 160.0);

        this->pre_calc_speed = return_val.gas;
        this->pre_calc_brake = return_val.brake;

        return return_val;
    }

    float get_target_speed()
    {
        return this->target_speed_;
    }
};

#endif // LON_CONTROLLER_HPP
