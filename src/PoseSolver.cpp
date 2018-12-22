/****************************************************************************
 *
 * This file is part of poseEstimation, a simple pose estimation test.
 *
 * Copyright (C) 2018 Frank Yang <yang0773@outlook.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *****************************************************************************/

#include <math.h>
#include "PoseSolver.hpp"

namespace posee {

void PoseSolver::setCameraMatrix(const Mat& cam_mat) {
  cam_matrix = cam_mat;
}

void PoseSolver::setDistCoefficients(const Mat& dist) {
  dist_coeff = dist;
}

} // posee
