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

}


using ARToolKitPlus::TrackerMultiMarker;

int main(int argc, char** argv) {
    //const int width = 320, height = 240, bpp = 1;
    //size_t numPixels = width * height * bpp;
    //size_t numBytesRead;
    //const char *fName = "data/markerboard_480-499.raw";
    const char *fName2 = "data/markerboard_480-499.jpg";
    //unsigned char cameraBuffer[numPixels];
/*
    // try to load a test camera image.
    // these images files are expected to be simple 8-bit raw pixel
    // data without any header. the images are expetected to have a
    // size of 320x240.
    if (FILE* fp = fopen(fName, "rb")) {
        numBytesRead = fread(cameraBuffer, 1, numPixels, fp);
        fclose(fp);
    } else {
        printf("Failed to open %s\n", fName);
        return -1;
    }

    if (numBytesRead != numPixels) {
        printf("Failed to read %s\n", fName);
        return -1;
    }
*/
    // opencv reads
    cv::Mat gsImage = cv::imread(fName2, cv::IMREAD_GRAYSCALE);
    cv::Mat image = cv::imread(fName2);

    int width = image.cols; int height = image.rows;
    uint8_t* cameraBuffer = new uint8_t[width*height];
    uint8_t* dPtr = cameraBuffer;
    for(int row = 0; row < height; row++)
      for(int col = 0; col < width; col++)
        *dPtr++ = gsImage.at<unsigned char>(row,col);


    if(image.empty())
    {
        printf("Error reading image!\n");
        exit(-2);
    }
    cv::cvtColor(image, image, CV_BGR2BGRA);

    // create a tracker that does:
    //  - 6x6 sized marker images (required for binary markers)
    //  - samples at a maximum of 6x6
    //  - works with luminance (gray) images
    //  - can load a maximum of 0 non-binary pattern
    //  - can detect a maximum of 8 patterns in one image
    TrackerMultiMarker tracker(width, height, 8, 6, 6, 6, 0);

    tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);

    // load a camera file.
    if (!tracker.init("data/PGR_M12x0.5_2.5mm.cal", "data/markerboard_480-499.cfg", 1.0f, 1000.0f)) {
        printf("ERROR: init() failed\n");
        return -1;
    }

    Mat K = Mat::zeros(3, 3, CV_64F);
    K.at<double>(0,0) = 492.37011418801217 * 0.5;
    K.at<double>(1,1) = 493.25907621710394 * 0.5;
    K.at<double>(0,2) = 259.37876574697924 * 0.5;
    K.at<double>(1,2) = 213.46213029915586 * 0.5;
    K.at<double>(2,2) = 1.0;

    tracker.getCamera()->printSettings();

    // the marker in the BCH test image has a thiner border...
    tracker.setBorderWidth(0.125f);

    // set a threshold. we could also activate automatic thresholding
    tracker.activateAutoThreshold(true);
    //tracker.setThreshold(160);

    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker.setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker.setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);

    // do the OpenGL camera setup
    //glMatrixMode(GL_PROJECTION)
    //glLoadMatrixf(tracker.getProjectionMatrix());

    // here we go, just one call to find the camera pose
    int numDetected = tracker.calc(cameraBuffer);

    // use the result of calc() to setup the OpenGL transformation
    //glMatrixMode(GL_MODELVIEW)
    //glLoadMatrixf(tracker.getModelViewMatrix());

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

    std::cout << K << std::endl;
    std::cout << pose << std::endl;
    std::cout << c << std::endl;

   // std::cout << pose << std::endl;
   drawARCube(image.clone(), pose, K);
   drawCoordinateSystem(image.clone(), pose, K);
   waitKey();

    //===========================================================================

    printf("\n%d good Markers found and used for pose estimation.\nPose-Matrix:\n  ", numDetected);
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
    delete[] cameraBuffer;
    return 0;
}
