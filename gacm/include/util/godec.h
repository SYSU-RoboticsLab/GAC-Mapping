/**
* This file is part of GAC-Mapping.
*
* Copyright (C) 2020-2022 JinHao He, Yilin Zhu / RAPID Lab, Sun Yat-Sen University 
* 
* For more information see <https://github.com/SYSU-RoboticsLab/GAC-Mapping>
*
* GAC-Mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* GAC-Mapping is distributed to support research and development of
* Ground-Aerial heterogeneous multi-agent system, but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GAC-Mapping. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GODEC_H
#define GODEC_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <iostream>
#include <queue>

Eigen::MatrixXd reducerank(const Eigen::MatrixXd &m, int r)
{
//   Eigen::BDCSVD<Eigen::MatrixXd> decomposition(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd s = decomposition.singularValues().asDiagonal();
  Eigen::MatrixXd u = decomposition.matrixU();
  Eigen::MatrixXd v = decomposition.matrixV();
  Eigen::MatrixXd uslice = u.block(0, 0, u.rows(), r);
  Eigen::MatrixXd vslice = v.block(0, 0, v.rows(), r);
  Eigen::MatrixXd sslice = s.block(0, 0, r, r);
  Eigen::MatrixXd approx = uslice*sslice*vslice.transpose();
//   std::cout << "Original:" << std::endl << m << std::endl;
//   std::cout << "U:" << std::endl << u << std::endl;
//   std::cout << "V:" << std::endl << v << std::endl;
//   std::cout << "S:" << std::endl << s << std::endl;
//   std::cout << "recovered:" << std::endl << u*s*v.transpose() << std::endl;
//   std::cout << "U slice:" << std::endl << uslice << std::endl;
//   std::cout << "V slice:" << std::endl << vslice << std::endl;
//   std::cout << "S slice:" << std::endl << sslice << std::endl;
//   std::cout << "approximation:" << std::endl << approx << std::endl;

  return approx;
}

class MatElt {
public:
  std::pair<int, int> index;
  float val;
  MatElt(std::pair<int, int> p, float x) : index(p), val(x) { }
};

Eigen::MatrixXd thresh(const Eigen::MatrixXd &x, int k)
{
  auto cmp = [](MatElt left, MatElt right) { return (left.val < right.val);};
  std::priority_queue<MatElt, std::vector<MatElt>, decltype(cmp)> q(cmp);
  for (int i = 0; i < x.rows(); i++)
    {
      for (int j = 0; j < x.cols(); j++)
        {
          q.push(MatElt(std::make_pair(i, j), x(i, j)));
        }
    }

  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(x.rows(), x.cols());
  for (int i = 0; i < k; i++)
    {
      MatElt elt = q.top();
      q.pop();
      auto f = elt.index;
      float s = elt.val;
      a(f.first, f.second) = s;
    }

//   std::cout << "a:" << std::endl << a;

  return a;
}

Eigen::MatrixXd godec(const Eigen::MatrixXd &x, int r, int k, int num_iterations)
{
    Eigen::MatrixXd l(x.rows(), x.cols());
    Eigen::MatrixXd s(x.rows(), x.cols());

    for (int i = 0; i < x.rows(); i++)
        for (int j = 0; j < x.cols(); j++) {
        {
            l(i, j) = x(i, j);
            s(i, j) = 0.0;
        }
    }

    for (int i = 0; i < num_iterations; i++) {
            l = reducerank(x - s, r);
            s = thresh(x - l, k);
    }

    // return l + s;
    return l;
}
#endif