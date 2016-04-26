//
//  DetectLanes.hpp
//  HistogramSegmentation
//
//  Created by Maitham Dib on 13/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#ifndef DetectLanes_hpp
#define DetectLanes_hpp

#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "createFourPointsForIPM.hpp"
class detectLanes {
    int houghThreshold = 153;
    int procWidth = 1280;
    int procHeight = 720;
    int top_margin = 420;
    int bottom_margin = 600;
    int left_lane_left_margin = 0;
    int left_lane_right_margin = 625;
    int right_lane_left_margin = 625;
    int right_lane_right_margin = 1100;
    int height = 720;
    
    
public:
    cv::Mat createRectangleROI(cv::Mat TargetImg);
    cv::Mat createTrapezoidROI(cv::Mat TargetImg);
    cv::Mat applyAdaptiveThresh_FastNLDenoising_Canny(cv::Mat imgGray);
    cv::Mat processImgGetCanny(cv::Mat ROITargetImg);
    std::vector<cv::Vec2f> HoughTransformReturnLeftRightLane(cv::Mat imgCanny);
    cv::Point2f getVanishingPoint(std::vector<cv::Vec2f> leftandRightLane);
    void drawFinalLinesOnImage(cv::Mat image, std::vector<cv::Vec2f> lines);
    std::vector<cv::Vec2f> getLaneBeginningAndEndPoints(std::vector<cv::Vec2f> leftandRightLineThetaAndRow);
    cv::Mat createTrapezoidROIInverse(cv::Mat TargetImg , std::vector<cv::Vec2f> laneEndPoints, std::vector<int> IPMTopAndBotLimits);
    std::vector<cv::Vec2f> findLeftLane(cv::Mat imgCanny);
    std::vector<cv::Vec2f> findRightLane(cv::Mat imgCanny);
    void drawLeftLinesOnImage(cv::Mat &image, std::vector<cv::Vec2f> lines);
    void drawRightLinesOnImage(cv::Mat &image, std::vector<cv::Vec2f> lines);
    void drawBoundingBoxes(cv::Mat image);
    void drawHoughLines(cv::Mat imgCanny, cv::Mat outputImg);
    std::vector<cv::Vec2f> findLeftLaneUsingSobel(cv::Mat imgCanny);
    cv::Mat processSobel(cv::Mat frameBGR);
    std::vector<cv::Vec2f> findRightLaneUsingSobel(cv::Mat imgSobel);
    
private:
    cv::Mat RectangleROISpecifyPoints(cv::Mat TargetImg, std::vector<int> IPMTopAndBot);
    std::vector<float> decideLanes(std::string laneDescriptor,std::vector<cv::Vec2f> lines, int height,int topMargin,int leftMargin,int rightMargin, int bottomMargin);
    float findIntercept(int bottomMargin,float rho,float theta);
};

#endif /* DetectLanes_hpp */

