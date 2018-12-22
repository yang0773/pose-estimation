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

#ifndef __PNP_SOLVER_HPP__
#define __PNP_SOLVER_HPP__

#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "PoseSolver.hpp"

using namespace std;
using namespace cv;

namespace posee {

class PnPSolver : public PoseSolver {

public:

  PnPSolver() {}

  virtual ~PnPSolver() {
    printf("~PnPSolver() called");
  }

  virtual int solve(const vector<Point3f>& points_object,
                    const vector<Point2f>& points_image,
                    Mat& R, Mat& t, Mat& wcoords_cam,
                    METHOD method = CV_P3P);

private:

  void rotateByZ(double x, double y, double thetaz, double& outx, double& outy);
  void rotateByY(double x, double z, double thetay, double& outx, double& outz);
  void rotateByX(double y, double z, double thetax, double& outy, double& outz);
};

} // posee
#endif
