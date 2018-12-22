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

#if defined(linux)
#include <unistd.h>
#else
#include <io.h>
#include <direct.h>
#endif

#include <sys/stat.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

static void help() {
  // print a welcome message, and the OpenCV version
  cout << "\nA simple tool to take images for stereo camera, "
          "Current OpenCV version " << CV_VERSION << endl;
  cout << "\nHot keys: \n"
          "\tESC - quit the program\n"
          "\tENTER - capture image\n"
          "\tLEFT MOUSE button - capture image\n"
          "Images would be stored in ./pic\n" << endl;
}

static bool snapshot = false;
static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ ) {
  if(event == EVENT_LBUTTONDOWN) {
    cout << "Left mouse button pressed" << endl;
    snapshot = true;
  }
}

int main(int argc, char **argv) {
  CommandLineParser parser(argc, argv, "{help h||}");
  if (parser.has("help")) {
    help();
    return 0;
  }

  VideoCapture capL(0);
  VideoCapture capR(1);
  if(!capL.isOpened() || !capR.isOpened()) {
    cout << "Stereo cameras not open: " << capL.isOpened() << " | " << capR.isOpened() << endl;
    return 1;
  }

  //capL.set(CV_CAP_PROP_SETTINGS, 1);
  double rate = capL.get(CV_CAP_PROP_FPS);
  capL.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  capL.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
  capR.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  capR.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
  int width = capL.get(CV_CAP_PROP_FRAME_WIDTH);
  int height= capL.get(CV_CAP_PROP_FRAME_HEIGHT);
  cout<<"Camera, rate: " << rate << " (w, h)/(" << width << ", " << height << ")" <<endl;

  Mat frameL, frameR;
  bool stop(false);
  while(!stop) {
    double timestampL = capL.get(CV_CAP_PROP_POS_MSEC);
    if(!capL.read(frameL)) {
      cerr << endl << "Failed to read frames from camera L ... " << endl;
      break;
    }
    double timestampR = capR.get(CV_CAP_PROP_POS_MSEC);
    if(!capR.read(frameR)) {
      cerr << endl << "Failed to read frames from camera R ... " << endl;
      break;
    }
    namedWindow("cameraL", 0);
    setMouseCallback("cameraL", onMouse, 0);
    resizeWindow("cameraL", width >> 2, height >> 2);
    imshow("cameraL", frameL);

    namedWindow("cameraR", 0);
    setMouseCallback("cameraR", onMouse, 0);
    resizeWindow("cameraR", width >> 2, height >> 2);
    imshow("cameraR", frameR);

    char c = (char)waitKey(10);
    if(c == 27) break; // ESC key value
    switch(c) {
    case 13: // ENTER key value
      cout << "ENTER key pressed" << endl;
      snapshot = true;
      break;
    }

    if (snapshot) {
      static int count = 0;
      snapshot = false;

      // create directory if not existed
      string file_dir = "pic";
      #if defined(linux)
      if (access(file_dir.c_str(), 0) == -1) {
        int flag = mkdir(file_dir.c_str(), 0777);
      #else
      if (_access(file_dir.c_str(), 0) == -1) {
        int flag = mkdir(file_dir.c_str());
      #endif
        if(flag == 0) cout << "OK to create dir: " << file_dir << endl;
        else cout << "Failed to create dir: " << file_dir << endl;
      }

      // save images with format jpg
      string file_index;
      if (count == 0) file_index = "000";
      else if (count < 10) file_index = "00" + to_string(count);
      else if (count < 100) file_index = "0" + to_string(count);
      string tsStringL = to_string(timestampL);
      string tsStringR = to_string(timestampR);
      string file_path_l = file_dir + "/calib_" + file_index + "_l_" + tsStringL + ".jpg";
      string file_path_r = file_dir + "/calib_" + file_index + "_r_" + tsStringR + ".jpg";
      cout << "saved to " << file_path_l  << endl;
      cout << "saved to " << file_path_r << endl;
      imwrite(file_path_l, frameL);
      imwrite(file_path_r, frameR);

      count++;
    }
  }

  return 0;
}
