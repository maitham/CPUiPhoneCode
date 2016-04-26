//
//  cameraCalibrationGetIntrinsicMatrix.cpp
//  ObjectDetectionSwift
//
//  Created by Maitham Dib on 04/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#include "cameraCalibrationGetIntrinsicMatrix.hpp"


using namespace cv;
using namespace std;

Mat getInstrinsicMatrix(Mat image)
{
    int numBoards = 1;
    int numCornersHor =9;
    int numCornersVer=6;
    
    int numSquares = numCornersHor * numCornersVer;
    cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);
    
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<cv::Point2f> corners;
    int successes=0;
    
    Mat gray_image;
    
    std::vector<cv::Point3f> obj;
    for(int j=0;j<numSquares;j++){
        obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));
    }
    
    while(successes<numBoards)
    {
        cvtColor(image, gray_image, CV_BGR2GRAY);
        bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        
        if(found)
        {
            cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray_image, board_sz, corners, found);
        }
//        imshow("win1", image);
//        imshow("win2", gray_image);
        
        
        image_points.push_back(corners);
        object_points.push_back(obj);
            
        printf("Snap stored!");
        
        successes++;
            
        if(successes>=numBoards)
        {
            Mat intrinsic = Mat(3, 3, CV_32FC1);
            Mat distCoeffs;
            vector<Mat> rvecs;
            vector<Mat> tvecs;
            
            intrinsic.ptr<float>(0)[0] = 1;
            intrinsic.ptr<float>(1)[1] = 1;
            
            calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
            return image;
        }
    }
    
    
    
//        Mat imageUndistorted;
//        while(1)
//        {
//            undistort(image, imageUndistorted, intrinsic, distCoeffs);
//            
//            imshow("win1", image);
//            imshow("win2", imageUndistorted);
//            waitKey(1);
//        }
    
    return image;
};

