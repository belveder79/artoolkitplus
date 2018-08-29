/**
 * Copyright (C) 2010  ARToolkitPlus Authors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:
 *  Daniel Wagner
 *  Pavel Rojtberg
 */

// Simple example to demonstrate multi-marker tracking with ARToolKitPlus
// This sample does not open any graphics window. It just
// loads test images and shows use to use the ARToolKitPlus API.

#include <cstdio>
#include <ARToolKitPlus/TrackerMultiMarker.h>

// invert
#include <ARToolKitPlus/matrix.h>

#include <opencv2/opencv.hpp>

using namespace cv;
void drawCoordinateSystem(Mat view, Mat& pose, Mat& K)
{
    namedWindow( "AR View2", WINDOW_NORMAL);
    int size = 100; // mm likely
    Mat c1 = Mat(4,4,CV_64F);
    c1.at<double>(0,0) = size; c1.at<double>(1,0) = 0; c1.at<double>(2,0) = 0; c1.at<double>(3,0) = 1;
    c1.at<double>(0,1) = 0; c1.at<double>(1,1) = size; c1.at<double>(2,1) = 0; c1.at<double>(3,1) = 1;
    c1.at<double>(0,2) = 0; c1.at<double>(1,2) = 0; c1.at<double>(2,2) = size; c1.at<double>(3,2) = 1;
    c1.at<double>(0,3) = 0; c1.at<double>(1,3) = 0; c1.at<double>(2,3) = 0; c1.at<double>(3,3) = 1;

    Mat pc1 = pose * c1;
    Mat cp1 = K * pc1;

    Point2d pt1(cp1.at<double>(0,0)/cp1.at<double>(2,0),cp1.at<double>(1,0)/cp1.at<double>(2,0));
    Point2d pt2(cp1.at<double>(0,1)/cp1.at<double>(2,1),cp1.at<double>(1,1)/cp1.at<double>(2,1));
    Point2d pt3(cp1.at<double>(0,2)/cp1.at<double>(2,2),cp1.at<double>(1,2)/cp1.at<double>(2,2));
    Point2d cent(cp1.at<double>(0,3)/cp1.at<double>(2,3),cp1.at<double>(1,3)/cp1.at<double>(2,3));

    line(view,cent,pt1,Scalar(0,0,255),4); //x
    line(view,cent,pt2,Scalar(0,255,0),4); //y
    line(view,cent,pt3,Scalar(255,0,0),4); //z

    imshow("AR View2", view);
    view.release();
}
void drawARCube(Mat view, Mat& pose, Mat& K)
{

    namedWindow( "AR View", WINDOW_NORMAL);

    int size = 40; // mm likely
    Mat c1 = Mat(4,4,CV_64F);
    c1.at<double>(0,0) = -size/2; c1.at<double>(1,0) = -size/2; c1.at<double>(2,0) = 0; c1.at<double>(3,0) = 1;
    c1.at<double>(0,1) = size/2; c1.at<double>(1,1) = -size/2; c1.at<double>(2,1) = 0; c1.at<double>(3,1) = 1;
    c1.at<double>(0,2) = size/2; c1.at<double>(1,2) = size/2; c1.at<double>(2,2) = 0; c1.at<double>(3,2) = 1;
    c1.at<double>(0,3) = -size/2; c1.at<double>(1,3) = size/2; c1.at<double>(2,3) = 0; c1.at<double>(3,3) = 1;

    Mat pc1 = pose * c1;
    Mat cp1 = K * pc1;

    Point2d pt1(cp1.at<double>(0,0)/cp1.at<double>(2,0),cp1.at<double>(1,0)/cp1.at<double>(2,0));
    Point2d pt2(cp1.at<double>(0,1)/cp1.at<double>(2,1),cp1.at<double>(1,1)/cp1.at<double>(2,1));
    Point2d pt3(cp1.at<double>(0,2)/cp1.at<double>(2,2),cp1.at<double>(1,2)/cp1.at<double>(2,2));
    Point2d pt4(cp1.at<double>(0,3)/cp1.at<double>(2,3),cp1.at<double>(1,3)/cp1.at<double>(2,3));

    c1.at<double>(2,0) = size/2; c1.at<double>(2,1) = size/2; c1.at<double>(2,2) = size/2; c1.at<double>(2,3) = size/2;
    pc1 = pose * c1;
    cp1 = K * pc1;

    Point2d pt5(cp1.at<double>(0,0)/cp1.at<double>(2,0),cp1.at<double>(1,0)/cp1.at<double>(2,0));
    Point2d pt6(cp1.at<double>(0,1)/cp1.at<double>(2,1),cp1.at<double>(1,1)/cp1.at<double>(2,1));
    Point2d pt7(cp1.at<double>(0,2)/cp1.at<double>(2,2),cp1.at<double>(1,2)/cp1.at<double>(2,2));
    Point2d pt8(cp1.at<double>(0,3)/cp1.at<double>(2,3),cp1.at<double>(1,3)/cp1.at<double>(2,3));

    line(view,pt1,pt2,Scalar(255,255,0),2);
    line(view,pt1,pt4,Scalar(255,255,0),2);
    line(view,pt2,pt3,Scalar(255,255,0),2);
    line(view,pt3,pt4,Scalar(255,255,0),2);

    line(view,pt1,pt5,Scalar(255,255,0),2);
    line(view,pt2,pt6,Scalar(255,255,0),2);
    line(view,pt3,pt7,Scalar(255,255,0),2);
    line(view,pt4,pt8,Scalar(255,255,0),2);

    line(view,pt5,pt6,Scalar(255,255,0),2);
    line(view,pt5,pt8,Scalar(255,255,0),2);
    line(view,pt6,pt7,Scalar(255,255,0),2);
    line(view,pt7,pt8,Scalar(255,255,0),2);

    Point2d ptc(K.at<double>(0,2),K.at<double>(1,2));
    line(view,ptc,ptc,Scalar(255,255,255),2);

    imshow("AR View", view);
    view.release();

}

using ARToolKitPlus::TrackerMultiMarker;

void fillMarkerId0(int i, int id, ARToolKitPlus::ARMultiEachMarkerInfoT* marker)
{
  marker[i].patt_id = id; // 0 works!
  marker[i].width = 40; // mm
  marker[i].center[0] = marker[i].center[1] = 0.0;
  float vals [12] = {
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 30
  };
  for(int j = 0; j < 12; j++)
    marker[i].trans[j/4][(j%4)] = vals[j];
  TrackerMultiMarker::arUtilMatInv(marker[i].trans, marker[i].itrans);
  ARFloat wpos3d[4][2];
  wpos3d[0][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[0][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[1][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[1][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[2][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[2][1] = marker[i].center[1] - marker[i].width * 0.5f;
  wpos3d[3][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[3][1] = marker[i].center[1] - marker[i].width * 0.5f;
  for (int j = 0; j < 4; j++) {
      marker[i].pos3d[j][0] = marker[i].trans[0][0] * wpos3d[j][0] + marker[i].trans[0][1] * wpos3d[j][1]
              + marker[i].trans[0][3];
      marker[i].pos3d[j][1] = marker[i].trans[1][0] * wpos3d[j][0] + marker[i].trans[1][1] * wpos3d[j][1]
              + marker[i].trans[1][3];
      marker[i].pos3d[j][2] = marker[i].trans[2][0] * wpos3d[j][0] + marker[i].trans[2][1] * wpos3d[j][1]
              + marker[i].trans[2][3];
  }
}

void fillMarkerId1(int i, int id, ARToolKitPlus::ARMultiEachMarkerInfoT* marker)
{
  marker[i].patt_id = id; // 0 works!
  marker[i].width = 40; // mm
  marker[i].center[0] = marker[i].center[1] = 0.0;
  float vals [12] = {
      1,  0, 0, 0,
      0,  0, -1, -30,
      0,  1, 0, 0
  };
  for(int j = 0; j < 12; j++)
    marker[i].trans[j/4][(j%4)] = vals[j];
  TrackerMultiMarker::arUtilMatInv(marker[i].trans, marker[i].itrans);
  ARFloat wpos3d[4][2];
  wpos3d[0][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[0][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[1][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[1][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[2][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[2][1] = marker[i].center[1] - marker[i].width * 0.5f;
  wpos3d[3][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[3][1] = marker[i].center[1] - marker[i].width * 0.5f;
  for (int j = 0; j < 4; j++) {
      marker[i].pos3d[j][0] = marker[i].trans[0][0] * wpos3d[j][0] + marker[i].trans[0][1] * wpos3d[j][1]
              + marker[i].trans[0][3];
      marker[i].pos3d[j][1] = marker[i].trans[1][0] * wpos3d[j][0] + marker[i].trans[1][1] * wpos3d[j][1]
              + marker[i].trans[1][3];
      marker[i].pos3d[j][2] = marker[i].trans[2][0] * wpos3d[j][0] + marker[i].trans[2][1] * wpos3d[j][1]
              + marker[i].trans[2][3];
  }
}

void fillMarkerId2(int i, int id, ARToolKitPlus::ARMultiEachMarkerInfoT* marker)
{
  marker[i].patt_id = id; // 0 works!
  marker[i].width = 40; // mm
  marker[i].center[0] = marker[i].center[1] = 0.0;
  float vals [12] = {
      1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, -1, -30
  };
  for(int j = 0; j < 12; j++)
    marker[i].trans[j/4][(j%4)] = vals[j];
  TrackerMultiMarker::arUtilMatInv(marker[i].trans, marker[i].itrans);
  ARFloat wpos3d[4][2];
  wpos3d[0][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[0][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[1][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[1][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[2][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[2][1] = marker[i].center[1] - marker[i].width * 0.5f;
  wpos3d[3][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[3][1] = marker[i].center[1] - marker[i].width * 0.5f;
  for (int j = 0; j < 4; j++) {
      marker[i].pos3d[j][0] = marker[i].trans[0][0] * wpos3d[j][0] + marker[i].trans[0][1] * wpos3d[j][1]
              + marker[i].trans[0][3];
      marker[i].pos3d[j][1] = marker[i].trans[1][0] * wpos3d[j][0] + marker[i].trans[1][1] * wpos3d[j][1]
              + marker[i].trans[1][3];
      marker[i].pos3d[j][2] = marker[i].trans[2][0] * wpos3d[j][0] + marker[i].trans[2][1] * wpos3d[j][1]
              + marker[i].trans[2][3];
  }
}

void fillMarkerId3(int i, int id, ARToolKitPlus::ARMultiEachMarkerInfoT* marker)
{
  marker[i].patt_id = id; // 0 works!
  marker[i].width = 40; // mm
  marker[i].center[0] = marker[i].center[1] = 0.0;
  float vals [12] = {
    1,  0, 0, 0,
    0,  0, 1, 30,
    0,  -1, 0, 0
  };
  for(int j = 0; j < 12; j++)
    marker[i].trans[j/4][(j%4)] = vals[j];
  TrackerMultiMarker::arUtilMatInv(marker[i].trans, marker[i].itrans);
  ARFloat wpos3d[4][2];
  wpos3d[0][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[0][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[1][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[1][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[2][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[2][1] = marker[i].center[1] - marker[i].width * 0.5f;
  wpos3d[3][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[3][1] = marker[i].center[1] - marker[i].width * 0.5f;
  for (int j = 0; j < 4; j++) {
      marker[i].pos3d[j][0] = marker[i].trans[0][0] * wpos3d[j][0] + marker[i].trans[0][1] * wpos3d[j][1]
              + marker[i].trans[0][3];
      marker[i].pos3d[j][1] = marker[i].trans[1][0] * wpos3d[j][0] + marker[i].trans[1][1] * wpos3d[j][1]
              + marker[i].trans[1][3];
      marker[i].pos3d[j][2] = marker[i].trans[2][0] * wpos3d[j][0] + marker[i].trans[2][1] * wpos3d[j][1]
              + marker[i].trans[2][3];
  }
}

void fillMarkerId4(int i, int id, ARToolKitPlus::ARMultiEachMarkerInfoT* marker)
{
  marker[i].patt_id = id; // 0 works!
  marker[i].width = 40; // mm
  marker[i].center[0] = marker[i].center[1] = 0.0;
  float vals [12] = {
      0,   0, -1,  -30,
      0,  -1,  0,  0,
      -1,  0,  0,  0
  };
  for(int j = 0; j < 12; j++)
    marker[i].trans[j/4][(j%4)] = vals[j];
  TrackerMultiMarker::arUtilMatInv(marker[i].trans, marker[i].itrans);
  ARFloat wpos3d[4][2];
  wpos3d[0][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[0][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[1][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[1][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[2][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[2][1] = marker[i].center[1] - marker[i].width * 0.5f;
  wpos3d[3][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[3][1] = marker[i].center[1] - marker[i].width * 0.5f;
  for (int j = 0; j < 4; j++) {
      marker[i].pos3d[j][0] = marker[i].trans[0][0] * wpos3d[j][0] + marker[i].trans[0][1] * wpos3d[j][1]
              + marker[i].trans[0][3];
      marker[i].pos3d[j][1] = marker[i].trans[1][0] * wpos3d[j][0] + marker[i].trans[1][1] * wpos3d[j][1]
              + marker[i].trans[1][3];
      marker[i].pos3d[j][2] = marker[i].trans[2][0] * wpos3d[j][0] + marker[i].trans[2][1] * wpos3d[j][1]
              + marker[i].trans[2][3];
  }
}

void fillMarkerId5(int i, int id, ARToolKitPlus::ARMultiEachMarkerInfoT* marker)
{
  marker[i].patt_id = id; // 0 works!
  marker[i].width = 40; // mm
  marker[i].center[0] = marker[i].center[1] = 0.0;
  float vals [12] = {
      0,  0,  1,  30,
      0, -1,  0,  0,
      1,  0,  0,  0
  };
  for(int j = 0; j < 12; j++)
    marker[i].trans[j/4][(j%4)] = vals[j];
  TrackerMultiMarker::arUtilMatInv(marker[i].trans, marker[i].itrans);
  ARFloat wpos3d[4][2];
  wpos3d[0][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[0][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[1][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[1][1] = marker[i].center[1] + marker[i].width * 0.5f;
  wpos3d[2][0] = marker[i].center[0] + marker[i].width * 0.5f;
  wpos3d[2][1] = marker[i].center[1] - marker[i].width * 0.5f;
  wpos3d[3][0] = marker[i].center[0] - marker[i].width * 0.5f;
  wpos3d[3][1] = marker[i].center[1] - marker[i].width * 0.5f;
  for (int j = 0; j < 4; j++) {
      marker[i].pos3d[j][0] = marker[i].trans[0][0] * wpos3d[j][0] + marker[i].trans[0][1] * wpos3d[j][1]
              + marker[i].trans[0][3];
      marker[i].pos3d[j][1] = marker[i].trans[1][0] * wpos3d[j][0] + marker[i].trans[1][1] * wpos3d[j][1]
              + marker[i].trans[1][3];
      marker[i].pos3d[j][2] = marker[i].trans[2][0] * wpos3d[j][0] + marker[i].trans[2][1] * wpos3d[j][1]
              + marker[i].trans[2][3];
  }
}

#define VIDEO 1

int main(int argc, char** argv) {

#if VIDEO
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat image;
    cap >> image;
    int width = image.cols; int height = image.rows;
#else
    // opencv reads
    const char *fName2 = "board.jpg";
    cv::Mat gsImage = cv::imread(fName2, cv::IMREAD_GRAYSCALE);
    cv::Mat image = cv::imread(fName2);

    int width = image.cols; int height = image.rows;
    uint8_t* cameraBuffer = new uint8_t[width*height];
    uint8_t* dPtr = cameraBuffer;
    for(int row = 0; row < height; row++)
      for(int col = 0; col < width; col++)
        *dPtr++ = gsImage.at<unsigned char>(row,col);

    imshow("Image", image);
    cvWaitKey();

    if(image.empty())
    {
        printf("Error reading image!\n");
        exit(-2);
    }
    cv::cvtColor(image, image, CV_BGR2BGRA);
#endif



    // create a tracker that does:
    //  - 6x6 sized marker images (required for binary markers)
    //  - samples at a maximum of 6x6
    //  - works with luminance (gray) images
    //  - can load a maximum of 0 non-binary pattern
    //  - can detect a maximum of 8 patterns in one image
    TrackerMultiMarker tracker(width, height, 8, 6, 6, 6, 0);

    tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

    ARToolKitPlus::Camera* m_camera = new ARToolKitPlus::Camera();
  	m_camera->cc[0] = 291.022;
  	m_camera->cc[1] = 245.487;
  	m_camera->fc[0] = 578.122;
  	m_camera->fc[1] = 574.056;
  	m_camera->xsize = width;
  	m_camera->ysize = height;
  	m_camera->mat[0][0] = m_camera->fc[0];
  	m_camera->mat[1][1] = m_camera->fc[1];
  	m_camera->mat[0][2] = m_camera->cc[0];
  	m_camera->mat[1][2] = m_camera->cc[1];
  	m_camera->mat[2][2] = 1.0f;
  	m_camera->kc[0] = 0;
  	m_camera->kc[1] = 0;
  	m_camera->kc[2] = 0;
  	m_camera->kc[3] = 0;
  	m_camera->kc[4] = 0;
  	m_camera->undist_iterations = 0;

    Mat K = Mat::zeros(3, 3, CV_64F);
    K.at<double>(0,0) = m_camera->fc[0];
    K.at<double>(1,1) = m_camera->fc[1];
    K.at<double>(0,2) = m_camera->mat[0][2];
    K.at<double>(1,2) = m_camera->mat[1][2];
    K.at<double>(2,2) = 1.0;

    int num = 6;
    ARToolKitPlus::ARMultiMarkerInfoT* marker_info = (ARToolKitPlus::ARMultiMarkerInfoT *) malloc(sizeof(ARToolKitPlus::ARMultiMarkerInfoT));
    ARToolKitPlus::ARMultiEachMarkerInfoT* marker = (ARToolKitPlus::ARMultiEachMarkerInfoT *) malloc(num * sizeof(ARToolKitPlus::ARMultiEachMarkerInfoT));
    fillMarkerId0(0,0,marker);
    fillMarkerId1(1,1,marker);
    fillMarkerId2(2,2,marker);
    fillMarkerId3(3,3,marker);
    fillMarkerId4(4,4,marker);
    fillMarkerId5(5,5,marker);

    marker_info->marker = marker;
    marker_info->marker_num = num;
    marker_info->prevF = 0;

    printf("Setting...\n");

    tracker.setMultiMarkerConfig(marker_info);
    tracker.setCamera(m_camera->clone(), 0.1f, 100.f);

    tracker.getCamera()->printSettings();

    // the marker in the test image has a thiner border...
    tracker.setBorderWidth(0.125f);

    // set a threshold. we could also activate automatic thresholding
    tracker.activateAutoThreshold(true);
    //tracker.setThreshold(160);

    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker.setUndistortionMode(ARToolKitPlus::UNDIST_NONE);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker.setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);//BCH);

    // do the OpenGL camera setup
    //glMatrixMode(GL_PROJECTION)
    //glLoadMatrixf(tracker.getProjectionMatrix());

#if VIDEO
    for(;;)
    {
      cap >> image; cv::Mat gsimage;
      cv::cvtColor(image, gsimage, CV_BGR2GRAY);

      imshow("Video", image);
      int numDetected = tracker.calc(gsimage.data);
#else
    // here we go, just one call to find the camera pose
    int numDetected = tracker.calc(cameraBuffer);
#endif
    // use the result of calc() to setup the OpenGL transformation
    //glMatrixMode(GL_MODELVIEW)
    //glLoadMatrixf(tracker.getModelViewMatrix());

    if(numDetected > 0) {
      //===========================================================================
      Mat pose = Mat::zeros(3, 4, CV_64F);
      ARFloat nMatrix[3][4];
      tracker.getARMatrix(nMatrix);
      // get pose
      for(int row = 0; row < 3; row++)
        for(int col = 0; col < 4; col++)
          pose.at<double>(row,col) = nMatrix[row][col];

      Mat R = pose( cv::Rect( 0, 0, 3, 3 ) );
      Mat t = pose( cv::Rect( 3, 0, 1, 3 ) );
      Mat c = -R.t() * t;
#if !VIDEO
      std::cout << K << std::endl;
      std::cout << pose << std::endl;
      std::cout << c << std::endl;
#endif
     // center the text
     char buf[64]; sprintf(buf,"# Markers: %d",numDetected);
     std::string text(buf);
     int fontFace = CV_FONT_HERSHEY_SIMPLEX;
     double fontScale = 1;
     int thickness = 2;
     int baseline = 0;
     Size textSize = getTextSize(text, fontFace,
                            fontScale, thickness, &baseline);
     Point textOrg(60,40);
     putText(image, text, textOrg, fontFace, fontScale,
            Scalar::all(255), thickness, 8);
/*
    // std::cout << pose << std::endl;
    FILE *f = fopen("points.m","wt");
    fprintf(f,"Pts = [\n");
    for(int x = 0; x < num; x++)
      for(int y = 0; y < 4; y++)
    {
      fprintf(f,"%f %f %f\n",marker[x].pos3d[y][0],marker[x].pos3d[y][1],marker[x].pos3d[y][2]);
    }
    fprintf(f,"];\nfigure(1); clf; hold on;\n");
    fprintf(f,"plot3(Pts(1,1),Pts(1,2),Pts(1,3),'ro','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(2,1),Pts(2,2),Pts(1,3),'rx','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(3:4,1),Pts(3:4,2),Pts(3:4,3),'r.','MarkerSize',24);\n");
    fprintf(f,"plot3(Pts(5,1),Pts(5,2),Pts(5,3),'go','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(6,1),Pts(6,2),Pts(6,3),'gx','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(7:8,1),Pts(7:8,2),Pts(7:8,3),'g.','MarkerSize',24);\n");
    fprintf(f,"plot3(Pts(9,1),Pts(9,2),Pts(9,3),'bo','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(10,1),Pts(10,2),Pts(10,3),'bx','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(11:12,1),Pts(11:12,2),Pts(11:12,3),'b.','MarkerSize',24);\n");
    fprintf(f,"plot3(Pts(13,1),Pts(13,2),Pts(13,3),'yo','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(14,1),Pts(14,2),Pts(14,3),'yx','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(15:16,1),Pts(15:16,2),Pts(15:16,3),'y.','MarkerSize',24);\n");
    fprintf(f,"plot3(Pts(17,1),Pts(17,2),Pts(17,3),'co','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(18,1),Pts(18,2),Pts(18,3),'cx','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(19:20,1),Pts(19:20,2),Pts(19:20,3),'c.','MarkerSize',24);\n");
    fprintf(f,"plot3(Pts(21,1),Pts(21,2),Pts(21,3),'mo','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(22,1),Pts(22,2),Pts(22,3),'mx','MarkerSize',12);\n");
    fprintf(f,"plot3(Pts(23:24,1),Pts(23:24,2),Pts(23:24,3),'m.','MarkerSize',24);\n");
    fprintf(f,"axis equal; box on; grid on; hold off;\n");
    fclose(f);

    return 0;
*/
    drawARCube(image.clone(), pose, K);
    drawCoordinateSystem(image.clone(), pose, K);

     waitKey(10);
    }
    //===========================================================================
#if !VIDEO
    printf("\n%d good Markers found and used for pose estimation.\nPose-Matrix:\n  ", numDetected);
    if(numDetected > 0)
      for (int i = 0; i < 16; i++)
        printf("%.2f  %s", tracker.getModelViewMatrix()[i], (i % 4 == 3) ? "\n  " : "");

    bool showConfig = false;

    if (showConfig) {
        const ARToolKitPlus::ARMultiMarkerInfoT *artkpConfig = tracker.getMultiMarkerConfig();
        printf("%d markers defined in multi marker cfg\n", artkpConfig->marker_num);

        printf("marker matrices:\n");
        for (int multiMarkerCounter = 0; multiMarkerCounter < artkpConfig->marker_num; multiMarkerCounter++) {
            printf("marker %d, id %d:\n", multiMarkerCounter, artkpConfig->marker[multiMarkerCounter].patt_id);
            for (int row = 0; row < 3; row++) {
                for (int column = 0; column < 4; column++)
                    printf("%.2f  ", artkpConfig->marker[multiMarkerCounter].trans[row][column]);
                printf("\n");
            }
        }
    }
#endif

#if VIDEO
    }
#else
    image.release();
    gsImage.release();
    delete[] cameraBuffer;
#endif
    delete m_camera;
    //delete newconfig;
    return 0;
}
