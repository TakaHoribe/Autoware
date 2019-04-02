/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <qpOASES.hpp>
#include <cmath>

// namespace qpsolver
// {
// bool solveEigenLeastSquare(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec,
//                            Eigen::VectorXd &U);
// bool solveEigenLeastSquareLLT(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec,
//                               Eigen::VectorXd &U);
// bool solveQpoases(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, Eigen::VectorXd &U);

// bool solveByHotstart(qpOASES::SQProblem& solver, const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, Eigen::VectorXd &U, bool is_init);

// }

class QPSolverEigenLeastSquare
{
  public:
    QPSolverEigenLeastSquare();

    void init(const int max_iter);
    bool solve(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, const double &max_u, Eigen::VectorXd &U);
};

class QPSolverEigenLeastSquareLLT
{
  public:
    QPSolverEigenLeastSquareLLT();

    void init(const int max_iter);
    bool solve(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, const double &max_u, Eigen::VectorXd &U);
};

class QPSolverQpoasesHotstart
{
  public:
    bool is_init_;
    int max_iter_;
    qpOASES::SQProblem solver_;
    QPSolverQpoasesHotstart();

    void init(const int max_iter);
    bool solve(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, const double &max_u, Eigen::VectorXd &U);
};