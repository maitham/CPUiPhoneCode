//
//  OpticFlow.hpp
//  LaneCuttingAversion
//
//  Created by Maitham Dib on 18/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#ifndef OpticFlow_hpp
#define OpticFlow_hpp

#include <stdio.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

cv::Mat drawOpticFlowArrows(int numberOfFeatures, std::vector<unsigned char> opticalFlowFoundFeature, std::vector<cv::Point2f> frame1Features ,std::vector<cv::Point2f> frame2Features,cv::Mat frame1);

cv::Mat calcOpticFlowAndDraw(cv::Mat frame1_1C,cv::Mat frame2_1C, cv::Mat frame1);
cv::Mat getDenseOpticFlow(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn);
cv::Mat getDenseOpticFlowRobust(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn,std::string leftOrRightLane);
cv::Mat getDenseOpticFlowRobustLeft(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn);
cv::Mat getDenseOpticFlowRobustRight(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn);

#endif /* OpticFlow_hpp */
