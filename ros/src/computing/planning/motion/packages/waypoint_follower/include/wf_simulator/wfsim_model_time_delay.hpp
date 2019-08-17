/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include "wf_simulator/wfsim_model_interface.hpp"

#include <iostream>
#include <queue>
#include <deque>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

class WFSimModelTimeDelayTwist : public WFSimModelInterface
{
public:
    WFSimModelTimeDelayTwist(double vel_lim, double angvel_lim, double dt, double vx_delay,
                             double vx_time_constant, double wz_delay, double wz_time_constant);
    ~WFSimModelTimeDelayTwist() = default;

private:
    const double MIN_TIME_CONSTANT;

    enum IDX
    {
        X = 0,
        Y,
        YAW,
        VX,
        WZ,
    };
    enum IDX_U
    {
        VX_DES = 0,
        WZ_DES,
    };

    const double vx_lim_;
    const double wz_lim_;

    std::deque<double> vx_input_queue_;
    std::deque<double> wz_input_queue_;
    const double vx_delay_;
    const double vx_time_constant_;
    const double wz_delay_;
    const double wz_time_constant_;

    void initializeInputQueue(const double &dt);

    double getX() override;
    double getY() override;
    double getYaw() override;
    double getVx() override;
    double getWz() override;
    double getSteer() override;
    Eigen::VectorXd calcModel(const Eigen::VectorXd &state, const Eigen::VectorXd &input) override;
};


class WFSimModelTimeDelaySteer : public WFSimModelInterface
{
public:
    WFSimModelTimeDelaySteer(double vel_lim, double steer_lim, double wheelbase, double dt, double vel_delay,
                             double vel_time_constant, double steer_delay, double steer_time_constant);
    ~WFSimModelTimeDelaySteer() = default;

private:
    const double MIN_TIME_CONSTANT;

    enum IDX
    {
        X = 0,
        Y,
        YAW,
        VX,
        STEER,
    };
    enum IDX_U
    {
        VX_DES = 0,
        STEER_DES,
    };

    const double vel_lim_;
    const double steer_lim_;
    const double wheelbase_;

    std::deque<double> vel_input_queue_;
    std::deque<double> steer_input_queue_;
    const double vel_delay_;
    const double vel_time_constant_;
    const double steer_delay_;
    const double steer_time_constant_;

    void initializeInputQueue(const double &dt);

    double getX() override;
    double getY() override;
    double getYaw() override;
    double getVx() override;
    double getWz() override;
    double getSteer() override;
    Eigen::VectorXd calcModel(const Eigen::VectorXd &state, const Eigen::VectorXd &input) override;
};
