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

#include <opencv2/opencv.hpp>
#include <PnPSolver.hpp>

using namespace std;
using namespace cv;
using namespace posee;

int main(int argc, char **argv)
{
  const float cam[] = {
    1148.313599, 0.000000, 644.061768,
    0.000000, 1148.111328, 460.901062,
    0.000000, 0.000000, 1.000000};

  const float dist[] = {
    -0.081629, 0.400191, -0.000493, -0.001383, 0.042124};

  Mat camMatrix(3, 3, CV_32FC1, (void*)cam);
  Mat distCoeff(5, 1, CV_32FC1, (void*)dist);

  PnPSolver solver;
  solver.setCameraMatrix(camMatrix);
  solver.setDistCoefficients(distCoeff);

  Point2f p2d_o(860, 488);
  Point2f p2d_a(786, 483);
  Point2f p2d_b(791, 560);
  Point2f p2d_c(840, 533);

#if 1 // estimated values
  //Point3f p3d_o(0.191254, 0.00830653, 1.08311);
  //Point3f p3d_a(0.121604, 0.00360444, 1.0821);
  //Point3f p3d_b(0.126469, 0.0760006, 1.08357);
  //Point3f p3d_c(0.172552, 0.050622, 1.08368);
  Point3f p3d_o(0.20377,  0.0255764, 1.08361);
  Point3f p3d_a(0.133815, 0.0208379, 1.0826);
  Point3f p3d_b(0.138718, 0.0935712, 1.08407);
  Point3f p3d_c(0.184996, 0.0680844, 1.08418);

#else // percipio values
  Point3f p3d_o(0.206085, 0.014937, 1.083359);
  Point3f p3d_a(0.133608, 0.010520, 1.083647);
  Point3f p3d_b(0.139000, 0.085870, 1.083305);
  Point3f p3d_c(0.187185, 0.059229, 1.085240);
#endif

  cv::Mat im = cv::imread("boxes.jpg");
  vector<Point3f> points_obj;
  points_obj.push_back(p3d_o - p3d_o); // original point
  points_obj.push_back(p3d_a - p3d_o);
  points_obj.push_back(p3d_b - p3d_o);
  points_obj.push_back(p3d_c - p3d_o);
  vector<Point2f> points_img;
  points_img.push_back(p2d_o);
  points_img.push_back(p2d_a);
  points_img.push_back(p2d_b);
  points_img.push_back(p2d_c);
  
  cout << endl << "Cam Matrix " << endl << camMatrix << endl;
  cout << endl << "Dist Coeff " << endl << distCoeff << endl;

  Mat rvec(3, 1, CV_32F);
  Mat tvec(3, 1, CV_32F);
  Mat wcoords_cam(3, 1, CV_32F);
  solver.solve(points_obj, points_img, rvec, tvec, wcoords_cam);

  // rotation vector --> matrix
  Mat rmat(3, 3, CV_32F);
  Rodrigues(rvec, rmat);
  
  cout << endl << "r vector " << endl << rvec << endl;
  cout << endl << "R matrix " << endl << rmat << endl;
  cout << endl << "t vector " << endl << tvec << endl;
  cout << endl << "camera world coords " << endl << wcoords_cam << endl;

  vector<Point3f> coords_world;
  vector<Point2f> coords_image;
  coords_world.push_back(Point3f(0.0, 0.0, 0.0)); // O
  coords_world.push_back(Point3f(0.1, 0.0, 0.0)); // X
  coords_world.push_back(Point3f(0.0, 0.1, 0.0)); // Y
  coords_world.push_back(Point3f(0.0, 0.0, 0.1)); // Z
  
  projectPoints(coords_world, rvec, tvec, camMatrix, distCoeff, coords_image);
  cout << endl << "coords_world " << endl << coords_world << endl;
  cout << endl << "coords_image " << endl << coords_image << endl;
  
  for(int i=0; i < points_img.size(); i++)
  {
      circle(im, points_img[i], 3, Scalar(0,0,255), -1);
  }
  
  cv::line(im, coords_image[0], coords_image[1], cv::Scalar(0, 0, 255), 2); // OX-red
  cv::line(im, coords_image[0], coords_image[2], cv::Scalar(0, 255, 0), 2); // OY-green
  cv::line(im, coords_image[0], coords_image[3], cv::Scalar(255, 0, 0), 2); // OZ-blue
  
  cv::imshow("Output", im);
  cv::waitKey(0);
}
