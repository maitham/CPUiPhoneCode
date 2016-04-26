//
//  cameraCalibrationGetIntrinsicMatrix.hpp
//  ObjectDetectionSwift
//
//  Created by Maitham Dib on 04/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#ifndef cameraCalibrationGetIntrinsicMatrix_hpp
#define cameraCalibrationGetIntrinsicMatrix_hpp

#include "opencv2/calib3d/calib3d.hpp"
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#import "opencv2/highgui.hpp"
//#import <opencv2/videoio/cap_ios.h>
//#import <opencv2/objdetect.hpp>
//#import <opencv2/imgcodecs/ios.h>
#include <stdio.h>
#include <vector>

cv::Mat getInstrinsicMatrix(cv::Mat image);

#endif /* cameraCalibrationGetIntrinsicMatrix_hpp */
