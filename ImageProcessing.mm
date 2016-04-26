//
//  ImageProcessing.m
//  ObjectDetectionSwift
//
//  Created by Maitham Dib on 22/02/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#import "ObjectDetectionSwift-Bridging-Header.h"
#import <Foundation/Foundation.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#import "opencv2/highgui.hpp"
#import <opencv2/videoio/cap_ios.h>
#import <opencv2/objdetect.hpp>
#import <opencv2/imgcodecs/ios.h>
#import "DetectLanes.hpp"
#import "IPM.hpp"
#import "KalmanFilterOneLane.hpp"
#import "OpticFlow.hpp"


#define MAX_NUM_LINES	200
#define USE_PPHT


using namespace std;
using namespace cv;

@interface ImageProcessing(){
    cv::CascadeClassifier cascade;
}
@end

@implementation ImageProcessing : NSObject
- (id)init {
        self = [super init];
        
    // Get xml files
        NSBundle *bundle = [NSBundle mainBundle];
        NSString *path = [bundle pathForResource:@"cars3" ofType:@"xml"];
        std::string cascadeName = (char *)[path UTF8String];
        
        if(!cascade.load(cascadeName)) {
            return nil;
        }
    
        return self;
    }

- (UIImage *)recognizeCars:(UIImage *)image {
    //  Store Image into Mat
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat width = image.size.width;
    CGFloat height = image.size.height;
    
    cv::Mat mat(height, width, CV_8UC4);
    
    CGContextRef contextRef = CGBitmapContextCreate(mat.data,
                                                    width,
                                                    height,
                                                    8,
                                                    mat.step[0],
                                                    colorSpace,
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault);
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, width, height), image.CGImage);
    CGContextRelease(contextRef);
    
    
    // Use Haar Cascades to draw out cars
    std::vector<cv::Rect> cars;
    cascade.detectMultiScale(mat, cars,
                             1.05, 1,
                             CV_HAAR_SCALE_IMAGE,
                             cv::Size(70, 70));
    
    
   // Draw Rectangles 
    for (int i = 0; i < cars.size(); i++)
    {
        cv::Point pt1(cars[i].x + cars[i].width, cars[i].y+ cars[i].height);
        cv::Point pt2(cars[i].x, cars[i].y);
        cv:: rectangle(mat, pt1, pt2, CvScalar(0,255,0,0),1,8,0);

    }

    
    
    // cv::Mat -> UIImage
    UIImage *resultImage = MatToUIImage(mat);
    
    return resultImage;

}

string type2str(int type) {
    string r;
    
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    
    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }
    
    r += "C";
    r += (chans+'0');
    
    return r;
}



cv::Mat cvMatFromUIImage(UIImage * image)
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;
    
    cv::Mat cvMat(rows, cols, CV_8UC4); // 8 bits per component, 4 channels (color channels + alpha)
    
    CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to  data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[0],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault); // Bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
    CGContextRelease(contextRef);
    
    return cvMat;
}


//cv::Mat getSideLaneForOpticFlow1(cv::Mat originalMat)
//{
//    detectLanes laneDetection;
//    // get instance of detect lanes
//    cv::Mat imgCanny = laneDetection.processImgGetCanny(originalMat);
//    
//    cv::Mat inversePerspective;
//    
//    static vector<Vec2f> ppLeft;
//    static vector<Vec2f> updatedLineFrameLeft;
//    static vector<Vec2f> ppRight;
//    vector<Vec2f> updatedLineFrameRight;
//    
//    
//    //INITIALISE KALMAN
//    static bool isKalmanLeftInitialised;
//    if (!isKalmanLeftInitialised) {
//        vector<Vec2f> leftLane = laneDetection.findLeftLane(imgCanny);
//        if (leftLane.size()>0) {
//            KalmanFilterOneLane leftKF(leftLane);
//            ppLeft = leftKF.predictOneLane();
//            updatedLineFrameLeft = leftKF.updateOneLane(leftLane);
//            isKalmanLeftInitialised=true;
//        }
//    }
//    static bool isKalmanRightInitialised;
//    if (!isKalmanRightInitialised) {
//        vector<Vec2f> rightLane = laneDetection.findRightLane(imgCanny);
//        if (rightLane.size()>0) {
//            KalmanFilterOneLane rightKF(rightLane);
//            ppRight = rightKF.predictOneLane();
//            updatedLineFrameRight = rightKF.updateOneLane(rightLane);
//            isKalmanRightInitialised=true;
//        }
//    }
//    
//    //Pull Left Lane parameters
//    vector<Vec2f> leftLane = laneDetection.findLeftLane(imgCanny);
//    if(leftLane.size()==0 && isKalmanLeftInitialised)
//        leftLane = ppLeft;
//    
//    // Pull Right Lane Parameters
//    vector<Vec2f> rightLane = laneDetection.findRightLane(imgCanny);
//    if(rightLane.size()==0 && isKalmanRightInitialised)
//        rightLane = ppRight;
//    
//    
//    if(leftLane.size()>0 && rightLane.size()>0){
//        vector<Vec2f> lanesSeperate;
//        lanesSeperate.push_back(Vec2f(leftLane[0][0],leftLane[0][1]));
//        lanesSeperate.push_back(Vec2f(rightLane[0][0],rightLane[0][1]));
//        
//        //detect vanishing points of input video
//        cv::Point2f vanishingPoints(0,0);
//        vanishingPoints = laneDetection.getVanishingPoint(lanesSeperate);
//        cout<<"VP1: "<<vanishingPoints.x<<","<<vanishingPoints.y<<endl;
//        if (vanishingPoints.x!=0 || vanishingPoints.y!=0 ||vanishingPoints.x<1000||vanishingPoints.y<680)
//        {
//            //Get end Points of line endPoints
//            vector<Vec2f> endPoints = laneDetection.getLaneBeginningAndEndPoints(lanesSeperate);
//            trapeziumCoordinates trap = createFourPointsForIPM(vanishingPoints.x, vanishingPoints.y);
//            cvtColor(originalMat, originalMat,CV_BGR2GRAY);
//            cv::Mat sideLanesimg= getLeftAndRightLaneImage(originalMat, endPoints,vanishingPoints);
//            
//            // Resize image to fit and also copy in colour image to get Only one Output
//            inversePerspective = sideLanesimg.clone();
//        }
//    }
//  
//        return inversePerspective;
//}

//cv::Mat getSideLaneForOpticFlow2(cv::Mat originalMat)
//{
//    detectLanes laneDetection;
//    // get instance of detect lanes
//    cv::Mat imgCanny = laneDetection.processImgGetCanny(originalMat);
//    
//    cv::Mat inversePerspective;
//    
//    static vector<Vec2f> ppLeft;
//    static vector<Vec2f> updatedLineFrameLeft;
//    static vector<Vec2f> ppRight;
//    vector<Vec2f> updatedLineFrameRight;
//    
//    
//    //INITIALISE KALMAN
//    static bool isKalmanLeftInitialised;
//    if (!isKalmanLeftInitialised) {
//        vector<Vec2f> leftLane = laneDetection.findLeftLane(imgCanny);
//        if (leftLane.size()>0) {
//            KalmanFilterOneLane leftKF(leftLane);
//            ppLeft = leftKF.predictOneLane();
//            updatedLineFrameLeft = leftKF.updateOneLane(leftLane);
//            isKalmanLeftInitialised=true;
//        }
//    }
//    static bool isKalmanRightInitialised;
//    if (!isKalmanRightInitialised) {
//        vector<Vec2f> rightLane = laneDetection.findRightLane(imgCanny);
//        if (rightLane.size()>0) {
//            KalmanFilterOneLane rightKF(rightLane);
//            ppRight = rightKF.predictOneLane();
//            updatedLineFrameRight = rightKF.updateOneLane(rightLane);
//            isKalmanRightInitialised=true;
//        }
//    }
//    
//    //Pull Left Lane parameters
//    vector<Vec2f> leftLane = laneDetection.findLeftLane(imgCanny);
//    if(leftLane.size()==0 && isKalmanLeftInitialised)
//        leftLane = ppLeft;
//    
//    // Pull Right Lane Parameters
//    vector<Vec2f> rightLane = laneDetection.findRightLane(imgCanny);
//    if(rightLane.size()==0 && isKalmanRightInitialised)
//        rightLane = ppRight;
//    
//    
//    if(leftLane.size()>0 && rightLane.size()>0){
//        vector<Vec2f> lanesSeperate;
//        lanesSeperate.push_back(Vec2f(leftLane[0][0],leftLane[0][1]));
//        lanesSeperate.push_back(Vec2f(rightLane[0][0],rightLane[0][1]));
//        
//        //detect vanishing points of input video
//        cv::Point2f vanishingPoints(0,0);
//        vanishingPoints = laneDetection.getVanishingPoint(lanesSeperate);
//        cout<<"VP1: "<<vanishingPoints.x<<","<<vanishingPoints.y<<endl;
//        if (vanishingPoints.x!=0 || vanishingPoints.y!=0 ||vanishingPoints.x<1000||vanishingPoints.y<680)
//        {
//            //Get end Points of line endPoints
//            vector<Vec2f> endPoints = laneDetection.getLaneBeginningAndEndPoints(lanesSeperate);
//            trapeziumCoordinates trap = createFourPointsForIPM(vanishingPoints.x, vanishingPoints.y);
//            cvtColor(originalMat, originalMat,CV_BGR2GRAY);
//            cv::Mat sideLanesimg= getLeftAndRightLaneImage(originalMat, endPoints,vanishingPoints);
//                        
//            // Resize image to fit and also copy in colour image to get Only one Output
//            inversePerspective = sideLanesimg.clone();
//        }
//    }
//    
//    return inversePerspective;
//}
//



cv::Mat getLeftLaneIpmRobust(cv::Mat &frame, vector<Vec2f> endPoints, cv::Point2f vanishingPoints2){
    trapeziumCoordinates trapLeft = createFourPointsForIPMLeftLane(vanishingPoints2.x, vanishingPoints2.y);
    
    int lowerLeftLanePointx = endPoints[0][0];
    int lowerLeftLanePointy = endPoints[0][1];
    
    int upperLeftLanePointx = endPoints[1][0];
    int upperLeftLanePointy = endPoints[1][1];
    
    int upperLeftIpmPointx = (trapLeft.point3[0][0]/trapLeft.point3[3][0]);
    int upperLeftIpmPointy = (trapLeft.point3[1][0]/trapLeft.point3[3][0]);
    
    int lowerLeftIpmPointx = (trapLeft.point4[0][0]/trapLeft.point4[3][0]);
    int lowerLeftIpmPointy = (trapLeft.point4[1][0]/trapLeft.point4[3][0]);
    
    int upperLeftDiffx = abs(upperLeftLanePointx - upperLeftIpmPointx);
    int lowerLeftDiffx = abs(lowerLeftLanePointx - lowerLeftIpmPointx);
    
    int upperLeftDiffxNonAbs = (upperLeftLanePointx - upperLeftIpmPointx);
    int lowerLeftDiffxNonAbs = (lowerLeftLanePointx - lowerLeftIpmPointx);
    
    int upperLeftDiffy = abs(upperLeftLanePointy - upperLeftIpmPointy);
    int lowerLeftDiffy = abs(lowerLeftLanePointy - lowerLeftIpmPointy);

    if(upperLeftDiffx >150|| lowerLeftDiffx>150 || lowerLeftDiffy> 150|| upperLeftDiffy>150){
        cout<<"Reset framecount for Left lane recommended"<<endl;
//        shouldvanishingPointBeReset= true;
    }else{
//        shouldvanishingPointBeReset = false;
    }
    
    
    cv::Mat inversePerspectiveViewOfLeftSide2 = createIPMOfLeftSideLaneRobust(frame, trapLeft);
    
    return inversePerspectiveViewOfLeftSide2;
}
cv::Mat getRightLaneIpmRobust(cv::Mat &frame, vector<Vec2f> endPoints, cv::Point2f vanishingPoints2){
    trapeziumCoordinates trapRight = createFourPointsForIPMRightLane(vanishingPoints2.x, vanishingPoints2.y);
    
    int lowerRightLanePointx = endPoints[2][0];
    int lowerRightLanePointy = endPoints[2][1];
    
    int upperRightLanePointx = endPoints[3][0];
    int upperRightLanePointy = endPoints[3][1];
    
    int upperRightIpmPointx = (trapRight.point3[0][0]/trapRight.point3[3][0]);
    int upperRightIpmPointy = (trapRight.point3[1][0]/trapRight.point3[3][0]);
    
    int lowerRightIpmPointx = (trapRight.point4[0][0]/trapRight.point4[3][0]);
    int lowerRightIpmPointy = (trapRight.point4[1][0]/trapRight.point4[3][0]);
    
    int upperRightDiffx = abs(upperRightLanePointx - upperRightIpmPointx);
    int lowerRightDiffx = abs(lowerRightLanePointx - lowerRightIpmPointx);
    
    int upperRightDiffy = abs(upperRightLanePointy - upperRightIpmPointy);
    int lowerRightDiffy = abs(lowerRightLanePointy - lowerRightIpmPointy);
    
    
    if(upperRightDiffx>150|| lowerRightDiffx>150 || lowerRightDiffy>150 || upperRightDiffy>150 ){
        cout<<"Reset framecount for right lane recommended"<<endl;
//        shouldvanishingPointBeReset = true;
    }else{
//        shouldvanishingPointBeReset = false;
    }
    
    cv::Mat inversePerspectiveViewOfRightSide2 = createIPMOfRightSideLaneRobust(frame, trapRight);
    return inversePerspectiveViewOfRightSide2;
}

cv::Mat mergeLeftAndRightImage(cv::Mat inversePerspectiveViewOfLeftSide2, cv::Mat inversePerspectiveViewOfRightSide2 ){
    
    //    string ty =  type2str( inversePerspectiveViewOfLeftSide2.type() );
    //    printf("MatrixLeft: %s %dx%d \n", ty.c_str(), inversePerspectiveViewOfLeftSide2.cols, inversePerspectiveViewOfLeftSide2.rows );
    //    string ty2 =  type2str( inversePerspectiveViewOfRightSide2.type() );
    //    printf("MatrixRight: %s %dx%d \n", ty2.c_str(), inversePerspectiveViewOfRightSide2.cols, inversePerspectiveViewOfRightSide2.rows );
    //
    Mat sideLanesimg2(Mat(720, 1280, CV_8UC3));
    sideLanesimg2 = cv::Scalar(255);    // or the desired uint8_t value from 0-255
    
    //    string ty3 =  type2str( sideLanesimg2.type() );
    //    printf("MatrixMerge: %s %dx%d \n", ty3.c_str(), sideLanesimg2.cols, sideLanesimg2.rows );
    
    inversePerspectiveViewOfLeftSide2.copyTo(sideLanesimg2(cv::Rect(0,0, 240,720)));
    inversePerspectiveViewOfRightSide2.copyTo(sideLanesimg2(cv::Rect(1040,0, 240,720)));
    return sideLanesimg2;
}


- (UIImage *)detectLanesEffeciently:(UIImage *)image :(UIImage *)image2{
    //Initialise LaneDetection
    detectLanes laneDetection;
    
    //  Store Image into Mat1
    cv::Mat frame1= cvMatFromUIImage(image);
    cv::Mat frame2 = cvMatFromUIImage(image2);
    
    //Get sobel image
    cv::Mat imgSobelFrame1 = laneDetection.processSobel(frame1);
    cv::Mat imgSobelFrame2 = laneDetection.processSobel(frame2);

    // Find both Left and Right Frames Frame 1 & 2
    vector<Vec2f> leftLaneFrame1 = laneDetection.findLeftLaneUsingSobel(imgSobelFrame1);
    vector<Vec2f> rightLaneFrame1 = laneDetection.findRightLaneUsingSobel(imgSobelFrame1);
    
    vector<Vec2f> leftLaneFrame2 = laneDetection.findLeftLaneUsingSobel(imgSobelFrame2);
    vector<Vec2f> rightLaneFrame2 = laneDetection.findRightLaneUsingSobel(imgSobelFrame2);

    
    //Initialise UImage
    UIImage *resultImage;

    
    if(leftLaneFrame1.size()>0 && rightLaneFrame1.size()>0 && leftLaneFrame2.size()>0 && rightLaneFrame2.size()>0 ){
        // Put lanes in one vector frame 1 & 2
        vector<Vec2f> detectedLanes1;
        detectedLanes1.push_back(Vec2f(leftLaneFrame1[0][0],leftLaneFrame1[0][1]));
        detectedLanes1.push_back(Vec2f(rightLaneFrame1[0][0],rightLaneFrame1[0][1]));
        
        vector<Vec2f> detectedLanes2;
        detectedLanes2.push_back(Vec2f(leftLaneFrame2[0][0],leftLaneFrame2[0][1]));
        detectedLanes2.push_back(Vec2f(rightLaneFrame2[0][0],rightLaneFrame2[0][1]));
        
        // Get the endPoints for IPM estimation
        vector<Vec2f> endPoints1 = laneDetection.getLaneBeginningAndEndPoints(detectedLanes1);
        vector<Vec2f> endPoints2 = laneDetection.getLaneBeginningAndEndPoints(detectedLanes2);
        
        // Detect vanishing points of input video
        cv::Point2f initialVanishingPoint1(0,0);
        initialVanishingPoint1 = laneDetection.getVanishingPoint(detectedLanes1);
        
        cv::Point2f initialVanishingPoint2(0,0);
        initialVanishingPoint2 = laneDetection.getVanishingPoint(detectedLanes2);
        
        
        
            // get the INVERSE PERSPECTIVE OF SIDE
            cv::Mat ipmOfLeftLaneRobust1 = getLeftLaneIpmRobust(frame1, endPoints1, initialVanishingPoint1);
            cv::Mat ipmOfRightLaneRobust1 = getRightLaneIpmRobust(frame1, endPoints1, initialVanishingPoint1);
            
            cv::Mat ipmOfLeftLaneRobust2 = getLeftLaneIpmRobust(frame2, endPoints2, initialVanishingPoint2);
            cv::Mat ipmOfRightLaneRobust2 = getRightLaneIpmRobust(frame2, endPoints2, initialVanishingPoint2);
            
            cv::Mat leftLaneToDrawOnRobust =ipmOfLeftLaneRobust2.clone();
            cv::Mat rightLaneToDrawOnRobust= ipmOfRightLaneRobust2.clone();
            
            //  Convert Left to gray for optic Flow and Denoise LEFT LANE
            cvtColor(ipmOfLeftLaneRobust1, ipmOfLeftLaneRobust1, CV_BGR2GRAY);
            cvtColor(ipmOfLeftLaneRobust2, ipmOfLeftLaneRobust2, CV_BGR2GRAY);
            cvtColor(ipmOfRightLaneRobust1, ipmOfRightLaneRobust1, CV_BGR2GRAY);
            cvtColor(ipmOfRightLaneRobust2, ipmOfRightLaneRobust2, CV_BGR2GRAY);
            
            cv::Mat ipmOfLeftLaneRobustDest1,ipmOfLeftLaneRobustDest2,ipmOfRightLaneRobustDest1,ipmOfRightLaneRobustDest2;
            
            bilateralFilter(ipmOfLeftLaneRobust1, ipmOfLeftLaneRobustDest1, 8, 16, 4);
            bilateralFilter(ipmOfLeftLaneRobust2, ipmOfLeftLaneRobustDest2, 8, 16, 4);
            bilateralFilter(ipmOfRightLaneRobust1, ipmOfRightLaneRobustDest1, 8, 16, 4);
            bilateralFilter(ipmOfRightLaneRobust2, ipmOfRightLaneRobustDest2, 8, 16, 4);
            
            
            cv::Mat opticFlowImageLeft = getDenseOpticFlowRobustLeft(ipmOfLeftLaneRobustDest1, ipmOfLeftLaneRobustDest2, leftLaneToDrawOnRobust);
            cv::Mat opticFlowImageRight = getDenseOpticFlowRobustRight(ipmOfRightLaneRobustDest1, ipmOfRightLaneRobustDest2, rightLaneToDrawOnRobust);
        
//            string ty =  type2str( opticFlowImageLeft.type() );
//            printf("MatrixLeft: %s %dx%d \n", ty.c_str(), opticFlowImageLeft.cols, opticFlowImageLeft.rows );
//            string ty2 =  type2str( opticFlowImageRight.type() );
//            printf("MatrixRight: %s %dx%d \n", ty2.c_str(), opticFlowImageRight.cols, opticFlowImageRight.rows );
        
            cvtColor(opticFlowImageLeft, opticFlowImageLeft, CV_BGR2RGB);
            cvtColor(opticFlowImageRight, opticFlowImageRight, CV_BGR2RGB);

//            string ty3 =  type2str( opticFlowImageLeft.type() );
//            printf("MatrixLeft: %s %dx%d \n", ty3.c_str(), opticFlowImageLeft.cols, opticFlowImageLeft.rows );
//            string ty4 =  type2str( opticFlowImageRight.type() );
//            printf("MatrixRight: %s %dx%d \n", ty4.c_str(), opticFlowImageRight.cols, opticFlowImageRight.rows );
//        
            cv::Mat mergedDenseOpticRobust= mergeLeftAndRightImage(opticFlowImageLeft, opticFlowImageRight);
            
            // apply optic flow to both and
            cv::resize(frame2, frame2, CvSize(800,720));
        
//            string ty5 =  type2str(frame2.type() );
//            printf("MatrixRight: %s %dx%d \n", ty5.c_str(), frame2.cols, frame2.rows );
//
        
            cvtColor(frame2, frame2, CV_BGR2RGB);
            frame2.copyTo(mergedDenseOpticRobust(cv::Rect(240,0, 800,720)));
            resultImage = MatToUIImage(mergedDenseOpticRobust);
    }else{
        cout<<"vanishingPointNotFound"<<endl;
        laneDetection.drawBoundingBoxes(frame2);
        resultImage = MatToUIImage(frame2);
        
    }
    
    return resultImage;

}

    
    


- (UIImage *)detectLanesGetOpticFlow:(UIImage *)image{
    //Initialise LaneDetection
    detectLanes laneDetection;
    
    //  Store Image into Mat1
    cv::Mat currentFrame= cvMatFromUIImage(image);
    cv::Mat previousFrame =currentFrame.clone() ;

    //Initialise UImage
    UIImage *resultImage;
    
        //Get sobel image
        cv::Mat imgSobelFrame1 = laneDetection.processSobel(currentFrame);
        cv::Mat imgSobelFrame2 = laneDetection.processSobel(previousFrame);
        
        // Find both Left and Right Frames Frame 1 & 2
        vector<Vec2f> leftLaneFrame1 = laneDetection.findLeftLaneUsingSobel(imgSobelFrame1);
        vector<Vec2f> rightLaneFrame1 = laneDetection.findRightLaneUsingSobel(imgSobelFrame1);
        
        vector<Vec2f> leftLaneFrame2 = laneDetection.findLeftLaneUsingSobel(imgSobelFrame2);
        vector<Vec2f> rightLaneFrame2 = laneDetection.findRightLaneUsingSobel(imgSobelFrame2);
        
        if(leftLaneFrame1.size()>0 && rightLaneFrame1.size()>0 && leftLaneFrame2.size()>0 && rightLaneFrame2.size()>0 ){
            // Put lanes in one vector frame 1 & 2
            vector<Vec2f> detectedLanes1;
            detectedLanes1.push_back(Vec2f(leftLaneFrame1[0][0],leftLaneFrame1[0][1]));
            detectedLanes1.push_back(Vec2f(rightLaneFrame1[0][0],rightLaneFrame1[0][1]));
            
            vector<Vec2f> detectedLanes2;
            detectedLanes2.push_back(Vec2f(leftLaneFrame2[0][0],leftLaneFrame2[0][1]));
            detectedLanes2.push_back(Vec2f(rightLaneFrame2[0][0],rightLaneFrame2[0][1]));
            
            // Get the endPoints for IPM estimation
            vector<Vec2f> endPoints1 = laneDetection.getLaneBeginningAndEndPoints(detectedLanes1);
            vector<Vec2f> endPoints2 = laneDetection.getLaneBeginningAndEndPoints(detectedLanes2);
            
            // Detect vanishing points of input video
            cv::Point2f initialVanishingPoint1(0,0);
            initialVanishingPoint1 = laneDetection.getVanishingPoint(detectedLanes1);
            
            cv::Point2f initialVanishingPoint2(0,0);
            initialVanishingPoint2 = laneDetection.getVanishingPoint(detectedLanes2);
            
            // get the INVERSE PERSPECTIVE OF SIDE
            cv::Mat ipmOfLeftLaneRobust1 = getLeftLaneIpmRobust(currentFrame, endPoints1, initialVanishingPoint1);
            cv::Mat ipmOfRightLaneRobust1 = getRightLaneIpmRobust(currentFrame, endPoints1, initialVanishingPoint1);
            
            cv::Mat ipmOfLeftLaneRobust2 = getLeftLaneIpmRobust(previousFrame, endPoints2, initialVanishingPoint2);
            cv::Mat ipmOfRightLaneRobust2 = getRightLaneIpmRobust(previousFrame, endPoints2, initialVanishingPoint2);
            
            cv::Mat leftLaneToDrawOnRobust =ipmOfLeftLaneRobust2.clone();
            cv::Mat rightLaneToDrawOnRobust= ipmOfRightLaneRobust2.clone();
            
            //  Convert Left to gray for optic Flow and Denoise LEFT LANE
            cvtColor(ipmOfLeftLaneRobust1, ipmOfLeftLaneRobust1, CV_BGR2GRAY);
            cvtColor(ipmOfLeftLaneRobust2, ipmOfLeftLaneRobust2, CV_BGR2GRAY);
            cvtColor(ipmOfRightLaneRobust1, ipmOfRightLaneRobust1, CV_BGR2GRAY);
            cvtColor(ipmOfRightLaneRobust2, ipmOfRightLaneRobust2, CV_BGR2GRAY);
            
            cv::Mat ipmOfLeftLaneRobustDest1,ipmOfLeftLaneRobustDest2,ipmOfRightLaneRobustDest1,ipmOfRightLaneRobustDest2;
            
            bilateralFilter(ipmOfLeftLaneRobust1, ipmOfLeftLaneRobustDest1, 8, 16, 4);
            bilateralFilter(ipmOfLeftLaneRobust2, ipmOfLeftLaneRobustDest2, 8, 16, 4);
            bilateralFilter(ipmOfRightLaneRobust1, ipmOfRightLaneRobustDest1, 8, 16, 4);
            bilateralFilter(ipmOfRightLaneRobust2, ipmOfRightLaneRobustDest2, 8, 16, 4);
            
            
            cv::Mat opticFlowImageLeft= getDenseOpticFlowRobustLeft(ipmOfLeftLaneRobustDest1, ipmOfLeftLaneRobustDest2, leftLaneToDrawOnRobust);
            cv::Mat opticFlowImageRight= getDenseOpticFlowRobustRight(ipmOfRightLaneRobustDest1, ipmOfRightLaneRobustDest2, rightLaneToDrawOnRobust);
            
            cvtColor(opticFlowImageLeft, opticFlowImageLeft, CV_BGR2RGB);
            cvtColor(opticFlowImageRight, opticFlowImageRight, CV_BGR2RGB);
            
            cv::Mat mergedDenseOpticRobust= mergeLeftAndRightImage(opticFlowImageLeft, opticFlowImageRight);
            
            cv::Mat matBuffer = currentFrame.clone();
            
            laneDetection.drawLeftLinesOnImage(matBuffer, leftLaneFrame1);
            laneDetection.drawRightLinesOnImage(matBuffer, rightLaneFrame1);
            
            // apply optic flow to both and
            cv::resize(matBuffer, matBuffer, CvSize(800,720));
            
            //            string ty5 =  type2str(frame2.type() );
            //            printf("MatrixRight: %s %dx%d \n", ty5.c_str(), frame2.cols, frame2.rows );
        
            cvtColor(matBuffer, matBuffer, CV_BGR2RGB);
            matBuffer.copyTo(mergedDenseOpticRobust(cv::Rect(240,0, 800,720)));
            resultImage = MatToUIImage(mergedDenseOpticRobust);
            
            previousFrame=currentFrame;
        }else{
            cout<<"vanishingPointNotFound"<<endl;
            laneDetection.drawBoundingBoxes(currentFrame);
            resultImage = MatToUIImage(currentFrame);
        }
    return resultImage;
    
}

- (UIImage *)carryOutInversePerspectiveMap:(UIImage *)image {
    //  Store Image into Mat
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat width = image.size.width;
    CGFloat height = image.size.height;
    cv::Mat mat(height, width, CV_8UC4);
    
    CGContextRef contextRef = CGBitmapContextCreate(mat.data,
                                                    width,
                                                    height,
                                                    8,
                                                    mat.step[0],
                                                    colorSpace,
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault);
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, width, height), image.CGImage);
    CGContextRelease(contextRef);
    
    //*****************************************************************************Express region of Interest
    cv::Rect roi(0,0.4*height,width,0.6*height);
    mat = mat(roi);


    // The 4-points at the input image
    vector<Point2f> origPoints;
    origPoints.push_back( Point2f(0, height) );
    origPoints.push_back( Point2f(width, height) );
    origPoints.push_back( Point2f(width/2+30, 140) );
    origPoints.push_back( Point2f(width/2-50, 140) );
    
    // The 4-points correspondences in the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back( Point2f(0, height) );
    dstPoints.push_back( Point2f(width, height) );
    dstPoints.push_back( Point2f(width, 0) );
    dstPoints.push_back( Point2f(0, 0) );
    
    IPM ipm( cv::Size(width, height), cv::Size(width, height), origPoints, dstPoints);
    
    Mat inputImgGray;
    Mat outputImg;
    
    // Color Conversion
    if(mat.channels() == 3)
        cvtColor(mat, inputImgGray, CV_BGR2GRAY);
    else
        cvtColor(mat, inputImgGray, CV_BGR2GRAY);
//        mat.copyTo(inputImgGray);
    
    // Process
    clock_t begin = clock();
    ipm.applyHomography( mat, outputImg );
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    printf("%.2f (ms)\r", 1000*elapsed_secs);
    ipm.drawPoints(origPoints, mat );

    UIImage *resultImage = MatToUIImage(outputImg);
    
    return resultImage;
}

@end