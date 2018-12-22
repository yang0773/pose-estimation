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

#ifndef __POSE_SOLVER_HPP__
#define __POSE_SOLVER_HPP__

#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace posee {

class PoseSolver {
public:

  PoseSolver() {}

  virtual ~PoseSolver() {
  }

  enum METHOD
  {
    SIMPLE,
    CV_P3P = SOLVEPNP_P3P,
  };

  void setCameraMatrix(const Mat& cam_mat);
  void setDistCoefficients(const Mat& dist);

  virtual int solve(const vector<Point3f>& points_object,
                    const vector<Point2f>& points_image,
                    Mat& R, Mat& t, Mat& wcoords_cam,
		    METHOD method) = 0;

protected:

  Mat cam_matrix;
  Mat dist_coeff;

private:

};

} // posee
#endif

