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

enum MODE {
  MODE_NORMAL = 0,
  MODE_PAUSE,
  MODE_STEP_ON,
  MODE_STEP_HALF,
} MODE;

static void help() {
  // print a welcome message, and the OpenCV version
  cout << "\nThis is a demo of mmatrix pose estimation, "
          "Current OpenCV version " << CV_VERSION << endl;
  cout << "It uses video by default, but you can provide a id(0/1/...) to camera.\n";
  cout << "\nParameters: \n"
          "\t$./vpose input.mp4 -c=config.yml -o=tracker.avi\n"
          "\t  -o=<tracker.avi>    # filename for written file\n"
          "\t  -c=<config.yml>     # intrinsic for camera\n";
  cout << "\nHot keys: \n"
          "\tESC - quit the program\n"
          "\tc - delete all the points\n"
          "\tp - pause/resume the current frame\n"
          "\tn - switch the \"night\" mode on/off\n"
          "\tm - step forward one by one frame\n"
          "\tf - save the original still frame to png\n"
          "\ti - save the over-drawed still frame to png\n"
          "\tr - trigger/pause/resume to record a video\n"
          "\tleft mouse button - add/remove a point when paused\n\n"
          "Commonly you are suggested to select image points clockwise,"
	  " beginning with left-up corner\n" << endl;
}

Point2f point;
bool addRemovePt = false;

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ ) {
  if (event == EVENT_LBUTTONDOWN) {
    point = Point2f((float)x, (float)y);
    cout << "clicking point: " << point << endl;
    addRemovePt = true;
  }
}

int main(int argc, char **argv)
{
  int imode = 0; // input source: 0 - video, 1 - camera, 2 - picture

  CommandLineParser parser(argc, argv,
      "{@input||}{help h||}{c||}{o|tracker.avi|}");
  if (parser.has("help")) {
    help();
    return 0;
  }

  int cam_id = -1;
  string input_file = parser.get<string>("@input");
  if (isdigit(input_file[0]) && input_file.size() == 1) {
    cam_id = parser.get<int>("@input");
    imode = 1;
  }

  string output_file;
  if (parser.has("o")) {
    output_file = parser.get<string>("o");
  } else output_file = "tracker.avi";

  // read intrinsic parameters
  string config_file;
  if (parser.has("c")) {
    config_file = parser.get<string>("c");
  } else {
    cout << "No config file found, refer to help" << endl;
    return 0;
  }
  cout << "config file: " << config_file << endl;
  cv::FileStorage fs(config_file, cv::FileStorage::READ);

  Mat camMatrix, distCoeff;
  fs["camera_matrix"] >> camMatrix;
  fs["distortion_coefficients"] >> distCoeff;
  cout << endl << "Cam Matrix " << endl << camMatrix << endl;
  cout << endl << "Dist Coeff " << endl << distCoeff << endl << endl;

  PnPSolver solver;
  solver.setCameraMatrix(camMatrix);
  solver.setDistCoefficients(distCoeff);

  // 4 points of rectangle with 8x6 grids
  vector<Point3f> points_obj;
  points_obj.push_back(Point3f(-100.0,  70.0, 0.0)); // left up corner
  points_obj.push_back(Point3f( 100.0,  70.0, 0.0)); // right up corner
  points_obj.push_back(Point3f( 100.0, -70.0, 0.0)); // right down corner
  points_obj.push_back(Point3f(-100.0, -70.0, 0.0)); // left down corner

  vector<Point3f> coords_world;
  coords_world.push_back(Point3f(  0.0,   0.0,   0.0)); // O
  coords_world.push_back(Point3f(100.0,   0.0,   0.0)); // X
  coords_world.push_back(Point3f(  0.0,  70.0,   0.0)); // Y
  coords_world.push_back(Point3f(  0.0,   0.0, 100.0)); // Z

  VideoCapture cap;
  if(imode == 0) cap.open(input_file);
  else if(imode == 1) cap.open(cam_id);

  if(!cap.isOpened()) {
    cout << "Can not open camera " << cam_id <<
            ", or video file: " << input_file << endl;
    return -1;
  }

  int fps = cap.get(CV_CAP_PROP_FPS);
  int image_w = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int image_h = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  int image_num = cap.get(CV_CAP_PROP_FRAME_COUNT);

  namedWindow("pose", 0);
  setMouseCallback("pose", onMouse, 0);
  resizeWindow("pose", image_w, image_h);

  int mode = MODE_NORMAL;
  bool stepDebug = false;
  bool nightMode = false;
  bool writeMode = false;
  VideoWriter writer;

  Mat gray_new, gray_cur, image, frame, frame_old;
  vector<Point2f> pts_cur;
  vector<Point2f> pts_new;
  TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
  Size winSize(31, 31);
  const int MAX_COUNT = 500;
  int frame_index = 0;

  // fetch a frame first
  cap >> frame;
  for(;;) {
    if(frame.empty()) break;

    frame_index++;
    frame.copyTo(image);
    cvtColor(image, gray_new, COLOR_BGR2GRAY);
    if(nightMode) image = Scalar::all(0);

    if(!pts_cur.empty()) {
      vector<uchar> status;
      vector<float> err;

      if(gray_cur.empty()) gray_new.copyTo(gray_cur);
      calcOpticalFlowPyrLK(gray_cur, gray_new, pts_cur, pts_new,
                           status, err, winSize, 3, termcrit, 0, 0.001);

      size_t i, k;
      for(i = k = 0; i < pts_new.size(); i++) {
        if(addRemovePt) {
          if(norm(point - pts_new[i]) <= 5) {
            addRemovePt = false;
            continue;
          }
        }

        if(!status[i]) continue;
        pts_new[k++] = pts_new[i];
        circle(image, pts_new[i], 3, Scalar(0,255,0), -1, 8);
      }
      pts_new.resize(k);
    }

    if(addRemovePt && pts_new.size() < (size_t)MAX_COUNT) {
      vector<Point2f> tmp;
      tmp.push_back(point);
      //cornerSubPix(gray_new, tmp, winSize, Size(-1,-1), termcrit);
      pts_new.push_back(tmp[0]);
      addRemovePt = false;
    }

    // target for 4 points PnP
    if(pts_new.size() == 4) {
      vector<Point2f> points_img;
      points_img.push_back(Point2f(pts_new[0]));
      points_img.push_back(Point2f(pts_new[1]));
      points_img.push_back(Point2f(pts_new[2]));
      points_img.push_back(Point2f(pts_new[3]));

      Mat rvec(3, 1, CV_32F);
      Mat tvec(3, 1, CV_32F);
      Mat wcoords_cam(3, 1, CV_32F);
      solver.solve(points_obj, points_img, rvec, tvec, wcoords_cam);
      // rotation vector --> matrix
      Mat rmat(3, 3, CV_32F);
      Rodrigues(rvec, rmat);

      vector<Point2f> coords_image;
      projectPoints(coords_world, rvec, tvec, camMatrix, distCoeff, coords_image);

      if(stepDebug) {
	    cout << endl << endl << "----- beginning of a frame -----" << endl;
        cout << "(x, y): " << pts_new[0] << endl;
        cout << "(x, y): " << pts_new[1] << endl;
        cout << "(x, y): " << pts_new[2] << endl;
        cout << "(x, y): " << pts_new[3] << endl;

        cout << endl << "r vector " << endl << rvec << endl;
        //cout << endl << "R matrix " << endl << rmat << endl;
        cout << endl << "t vector " << endl << tvec << endl;
        //cout << endl << "camera world coords " << endl << wcoords_cam << endl;
        //cout << endl << "coords_world " << endl << coords_world << endl;
        cout << endl << "coords_image " << endl << coords_image << endl;

        stepDebug = false;
      }

      // draw a simple coordinates system
      line(image, coords_image[0], coords_image[1], cv::Scalar(0, 0, 255), 2); // OX
      line(image, coords_image[0], coords_image[2], cv::Scalar(0, 255, 0), 2); // OY
      line(image, coords_image[0], coords_image[3], cv::Scalar(255, 0, 0), 2); // OZ
    }

    imshow("pose", image);

    if(writeMode) writer << image;

    char c = (char)waitKey(10);
    if(c == 27) break; // ESC key
    switch(c) {
    case 'c':
      pts_cur.clear();
      pts_new.clear();
      break;
    case 'p':
      if(mode == MODE_NORMAL) mode = MODE_PAUSE;
      else mode = MODE_NORMAL;
      break;
    case 'n':
      nightMode = !nightMode;
      if(mode == MODE_STEP_HALF) mode = MODE_STEP_ON;
      break;
    case 'm':
      mode = MODE_STEP_ON;
      break;
    case 'f':
      {
        string png = "frame-" + to_string(frame_index) + ".png";
        cout << "Saved the original frame to " << png << endl;
        imwrite(png, frame);
      }
      break;
    case 'i':
      {
        string png = "image-" + to_string(frame_index) + ".png";
        cout << "Saved the drawed frame to " << png << endl;
        imwrite(png, image);
      }
      break;
    case 'r':
      static int triggerW = 1;
      if (triggerW == 1) {
        triggerW = 0;
        writer = VideoWriter(output_file, CV_FOURCC('X', 'V', 'I', 'D'),
                             fps, Size(image_w, image_h), 1);
      }
      writeMode = !writeMode;
      break;
    }

    std::swap(pts_new, pts_cur);
    cv::swap(gray_cur, gray_new);

    if(mode == MODE_NORMAL || mode == MODE_STEP_ON) {
      cap >> frame;
      frame.copyTo(image);
      frame.copyTo(frame_old);
      cvtColor(image, gray_new, COLOR_BGR2GRAY);
      if(nightMode) image = Scalar::all(0);

      if(mode == MODE_STEP_ON) {
        mode = MODE_STEP_HALF;
        stepDebug = true;
      }
    } else if(mode == MODE_PAUSE) {
      frame_old.copyTo(frame);
    }
  }

  cap.release();
  waitKey(0);

  return 0;
}
