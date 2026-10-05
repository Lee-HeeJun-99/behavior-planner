#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cmath>

/*
제작 : 예원태
문의 : solnox99@koreatech.ac.kr

설명 :
2023년 창작차 대회에서 사용한 Control 코드의 main 코드이다.
여러가지 구조와 함수, PID 클래스를 정의한 코드이다.
최초에 PID 클래스를 위해 파일을 만들어 파일 이름이 "PIDController.hpp" 이지만,
어쩌다 보니 잡다한 창고가 되었다.
*/

struct Point {
    double x;
    double y;
};

struct GasAndBrake {
    double gas;
    double brake;
};

// 두 점을 연결하는 벡터의 외적 계산
double cross_product(Point p1, Point p2) {
    return p1.x * p2.y - p1.y * p2.x;
}

double determine_side(Point reference, Point target, Point origin) {
    double cross = 0.0;
    Point refVector = {reference.x - origin.x, reference.y - origin.y};
    Point targetVector = {target.x - origin.x, target.y - origin.y};

    return cross = cross_product(refVector, targetVector);
}

// min_val, max_val 사이로 value를 clip 한다.
template <typename T>
T clip(const T &value, const T &min_val, const T &max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

double low_pass_filter(double val, double pre_val, float alpha)
{
    return val * alpha + pre_val * (1-alpha);
}

double nomalize_angle(double rad_angle)
{
    double rad_ang = rad_angle;
    while (rad_ang > M_PI)
        rad_ang -= 2 * M_PI;
    while (rad_ang < -M_PI)
        rad_ang += 2 * M_PI;
    return rad_ang;
}


class PIDController
{
public:
    PIDController(double min_output, double max_output)
        : min_output_(min_output), max_output_(max_output),
          integral_(0), prev_error_(0), has_prev_(false)  {}

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        pre_derivative_ = 0.0;
        has_prev_ = false;
    }

    void set_PID_gain(double kp, double kd)
    {
        // i gain은 편의상 0.0으로 고정.
        kp_ = kp;
        ki_ = 0.0; // ki;
        kd_ = kd;
    }

    double compute(double target_value, double measured_value, float alpha, double dt = 0.05)
    {
        double error = target_value - measured_value;
        integral_ += error * dt;

        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        derivative = low_pass_filter(derivative, pre_derivative_, alpha);
        pre_derivative_ = derivative;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        if (output < min_output_)
        {
            output = min_output_;
        } else if (output > max_output_){
            output = max_output_;
        }

        return output;
    }

    float get_p_gain(){  // 디버깅용
        return kp_;
    }

    float get_d_gain(){  // 디버깅용
        return kd_;
    }

private:
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double min_output_;
    double max_output_;
    double integral_;
    double prev_error_;

    double pre_derivative_ = 0.0;
    bool   has_prev_;

};

#endif  // PID_CONTROLLER_HPP
