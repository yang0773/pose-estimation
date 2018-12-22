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

#include "PnPSolver.hpp"

namespace posee {

void PnPSolver::rotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
	double x1 = x;
	double y1 = y;
	double rz = thetaz * CV_PI / 180;
	outx = cos(rz) * x1 - sin(rz) * y1;
	outy = sin(rz) * x1 + cos(rz) * y1;
}

void PnPSolver::rotateByY(double x, double z, double thetay, double& outx, double& outz)
{
	double x1 = x;
	double z1 = z;
	double ry = thetay * CV_PI / 180;
	outx = cos(ry) * x1 + sin(ry) * z1;
	outz = cos(ry) * z1 - sin(ry) * x1;
}

void PnPSolver::rotateByX(double y, double z, double thetax, double& outy, double& outz)
{
	double y1 = y;
	double z1 = z;
	double rx = thetax * CV_PI / 180;
	outy = cos(rx) * y1 - sin(rx) * z1;
	outz = cos(rx) * z1 + sin(rx) * y1;
}

int PnPSolver::solve(const vector<Point3f>& points_object,
                     const vector<Point2f>& points_image,
                     Mat& r, Mat& t, Mat& wcoords_cam,
		             METHOD method)
{
  solvePnP(points_object, points_image, cam_matrix, dist_coeff, r, t, false, SOLVEPNP_ITERATIVE);

  Mat R(3, 3, CV_64F);
  Rodrigues(r, R);

  // solve the camera position in world coordinates
  double r11 = R.ptr<double>(0)[0];
  double r12 = R.ptr<double>(0)[1];
  double r13 = R.ptr<double>(0)[2];
  double r21 = R.ptr<double>(1)[0];
  double r22 = R.ptr<double>(1)[1];
  double r23 = R.ptr<double>(1)[2];
  double r31 = R.ptr<double>(2)[0];
  double r32 = R.ptr<double>(2)[1];
  double r33 = R.ptr<double>(2)[2];

  // solve euler, refer to http://www.gregslabaugh.net/publications/euler.pdf
  double thetaz = atan2(r21, r11) / CV_PI * 180;
  double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
  double thetax = atan2(r32, r33) / CV_PI * 180;

  double x = t.ptr<double>(0)[0];
  double y = t.ptr<double>(0)[1];
  double z = t.ptr<double>(0)[2];

  // rotate the point by Z->Y->X order
  rotateByZ(x, y, -1 * thetaz, x, y);
  rotateByY(x, z, -1 * thetay, x, z);
  rotateByX(y, z, -1 * thetax, y, z);

  wcoords_cam.ptr<float>(0)[0] = x * -1;
  wcoords_cam.ptr<float>(0)[1] = y * -1;
  wcoords_cam.ptr<float>(0)[2] = z * -1;

  return 0;
}

} // posee
