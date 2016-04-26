//
//  DetectLanes.cpp
//  HistogramSegmentation
//
//  Created by Maitham Dib on 13/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#include "DetectLanes.hpp"

using namespace cv;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*...................................................FILTER IMAGE.......................................................................*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create Rectangle ROI

cv::Mat detectLanes::applyAdaptiveThresh_FastNLDenoising_Canny(cv::Mat imgGray){
cv:Mat outputImage;
    fastNlMeansDenoising(imgGray, outputImage,10,7,21);
    //adaptive Thresholding
    adaptiveThreshold(outputImage, outputImage, 255,ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 7, 2);
    //    //removing any noise which was amplified due to thresholding
    fastNlMeansDenoising(outputImage, outputImage,10,7,21);
    cv::Canny(outputImage, outputImage, 180, 120, 3);
    fastNlMeansDenoising(imgGray, outputImage,10,7,21);
    return outputImage;
}

cv::Mat detectLanes::processImgGetCanny(cv::Mat TargetImg){
    cv::Size procSize = cv::Size(procWidth, procHeight);
    // Resize to processing size
    cv::resize(TargetImg, TargetImg, procSize);
    //Convert to GrayScale
    cv::Mat imgGRAY;
    //        TargetImg.copyTo(outputImg);
    cv::cvtColor(TargetImg, imgGRAY, CV_BGR2GRAY);
    imgGRAY = createRectangleROI(imgGRAY);
    
    // ++++++++++++++++++++++++++++++++++++++++
    // Process
    // ++++++++++++++++++++++++++++++++++++++++
    cv::Mat imgCanny;
    //    imgCanny = processImage(imgGRAY);
    cv::Canny(imgGRAY, imgCanny, 180, 120, 3);
    return imgCanny;
}

void detectLanes:: drawHoughLines(cv::Mat imgCanny, cv::Mat outputImg){
    vector<Vec2f> lines;
    cv::HoughLines(imgCanny, lines, 1, M_PI/180, 50,0,0);
    
    // Filter out vector according to the angles of the left and right lane.
    vector<Vec2f> filteredLines(lines.size());
    auto it = copy_if(lines.begin(), lines.end(),
                      filteredLines.begin(),
                      [](const Vec2f &v) {
                          float angle = v[1];
                          return (angle <= 140*CV_PI/180 && angle >=120*CV_PI/180) || (angle>55*CV_PI/180 && angle<70*CV_PI/180) ;
                      });
    filteredLines.resize(std::distance(filteredLines.begin(),it));
    
    for(size_t i=0; i< filteredLines.size(); i++)
    {
        float rho = filteredLines[i][0];
        float theta = filteredLines[i][1];
        
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        
        cv::Point pt1, pt2;
        pt1.x = cvRound(x0 + 1500*(-b));
        pt1.y = cvRound(y0 + 1500*(a));
        pt2.x = cvRound(x0 - 1500*(-b));
        pt2.y = cvRound(y0 - 1500*(a));
        
        line(outputImg, pt1, pt2, CV_RGB(0,0,0), 2);
    }
}


cv::Mat detectLanes::processSobel(cv::Mat frameBGR){
    cv::Mat imgGray;
    cvtColor(frameBGR, imgGray, CV_BGR2GRAY);
    cv::Mat imageSobel;
    cv::Sobel(imgGray, imageSobel, CV_8U, 1, 0, 3);
    cv::Mat imageSobel2;
    cv::Sobel(imageSobel, imageSobel2, CV_8U, 0, 1, 3);
    cv::Mat medianBlur;
    cv::medianBlur(imageSobel2, medianBlur, 3);
    cv::Mat finalOutput;
    cv::threshold(medianBlur, finalOutput, 100, 255, cv::THRESH_BINARY);
    return finalOutput;
}



vector<Vec2f> detectLanes::findLeftLaneUsingSobel(cv::Mat imgSobel){
    vector<Vec2f> lines;
    cv::HoughLines(imgSobel, lines, 1, M_PI/180, 50,0,0);
    // Filter out vector according to the angles of the left and right lane.
    vector<Vec2f> filteredLines(lines.size());
    auto it = copy_if(lines.begin(), lines.end(),
                      filteredLines.begin(),
                      [](const Vec2f &v) {
                          float angle = v[1];
                          return ((angle>55*CV_PI/180 && angle<70*CV_PI/180)) ;
                      });
    filteredLines.resize(std::distance(filteredLines.begin(),it));
    
    
    //Pull out the best left lane according to weighting algorithm
    vector<float> lanePara;
    vector<Vec2f> leftThetaAndRow;
    
    lanePara = decideLanes("left", filteredLines, height, top_margin, left_lane_left_margin, left_lane_right_margin, bottom_margin);
    if (lanePara.size()>0)
    {   //Pull out the intercept
        //Draw Left Lane on Image and intercept Point
        //                    cv::circle(outputImg, Point(lanePara[5] ,bottom_margin), 20, Scalar(255, 255, 0));
        //                    line(outputImg, Point(lanePara[0],lanePara[1]), Point(lanePara[2],lanePara[3]), CV_RGB(0, 255, 0), 10, 8);
        leftThetaAndRow.push_back(Vec2f(lanePara[6],lanePara[4])); // push back left lane rho and theta
        //            cout<<"laneLeft"<<lanePara[6]<<"  theta  = "<<lanePara[4]<<endl;
    }else{
        printf("Left lane cannot be found");
    }
    return leftThetaAndRow;
}

vector<Vec2f> detectLanes::findRightLaneUsingSobel(cv::Mat imgSobel){
    vector<Vec2f> lines;
    cv::HoughLines(imgSobel, lines, 1, M_PI/180, 50,0,0);
    // Filter out vector according to the angles of the left and right lane.
    vector<Vec2f> filteredLines(lines.size());
    auto it = copy_if(lines.begin(), lines.end(),
                      filteredLines.begin(),
                      [](const Vec2f &v) {
                          float angle = v[1];
                          return (angle <= 140*CV_PI/180 && angle >=120*CV_PI/180) ;
                      });
    filteredLines.resize(std::distance(filteredLines.begin(),it));
    
    //Pull out the best left lane according to weighting algorithm
    vector<Vec2f> rightThetaAndRow;
    
    //Pull out the best right lane according to weighting algorithm
    vector<float> lanePara2;
    lanePara2 = decideLanes("right", filteredLines, height, top_margin, right_lane_left_margin, right_lane_right_margin, bottom_margin);
    if (lanePara2.size()>0)
    {
        //Draw Right Lane on Image and intercept //Draw Intercept with right box on Image as blue circle
        //            line(outputImgToDrawOn, Point(lanePara2[0],lanePara2[1]), Point(lanePara2[2],lanePara2[3]), CV_RGB(0, 255, 0), 10, 8);
        //                    cv::circle(ou, Point(lanePara2[5] ,bottom_margin), 20, Scalar(255, 255, 0));
        rightThetaAndRow.push_back(Vec2f(lanePara2[6],lanePara2[4])); // push back right lane rho and theta;
        
    }
    return rightThetaAndRow ;
}









vector<Vec2f> detectLanes::findLeftLane(cv::Mat imgCanny){
    vector<Vec2f> lines;
    cv::HoughLines( imgCanny, lines, 1, CV_PI/180, houghThreshold);
    // Filter out vector according to the angles of the left and right lane.
    vector<Vec2f> filteredLines(lines.size());
    auto it = copy_if(lines.begin(), lines.end(),
                      filteredLines.begin(),
                      [](const Vec2f &v) {
                          float angle = v[1];
                          return ((angle>55*CV_PI/180 && angle<70*CV_PI/180)) ;
                      });
    filteredLines.resize(std::distance(filteredLines.begin(),it));
    
    
    //Pull out the best left lane according to weighting algorithm
    vector<float> lanePara;
    vector<Vec2f> leftThetaAndRow;
    
    lanePara = decideLanes("left", filteredLines, height, top_margin, left_lane_left_margin, left_lane_right_margin, bottom_margin);
    if (lanePara.size()>0)
    {   //Pull out the intercept
        //Draw Left Lane on Image and intercept Point
        //                    cv::circle(outputImg, Point(lanePara[5] ,bottom_margin), 20, Scalar(255, 255, 0));
        //                    line(outputImg, Point(lanePara[0],lanePara[1]), Point(lanePara[2],lanePara[3]), CV_RGB(0, 255, 0), 10, 8);
        leftThetaAndRow.push_back(Vec2f(lanePara[6],lanePara[4])); // push back left lane rho and theta
        //            cout<<"laneLeft"<<lanePara[6]<<"  theta  = "<<lanePara[4]<<endl;
    }else{
        printf("Left lane cannot be found");
    }
    return leftThetaAndRow;
}

vector<Vec2f> detectLanes::findRightLane(cv::Mat imgCanny){
    vector<Vec2f> lines;
    cv::HoughLines( imgCanny, lines, 1, CV_PI/180, houghThreshold);
    // Filter out vector according to the angles of the left and right lane.
    vector<Vec2f> filteredLines(lines.size());
    auto it = copy_if(lines.begin(), lines.end(),
                      filteredLines.begin(),
                      [](const Vec2f &v) {
                          float angle = v[1];
                          return (angle <= 160*CV_PI/180 && angle >=100*CV_PI/180) ;
                      });
    filteredLines.resize(std::distance(filteredLines.begin(),it));
    
    //Pull out the best left lane according to weighting algorithm
    vector<Vec2f> rightThetaAndRow;
    
    //Pull out the best right lane according to weighting algorithm
    vector<float> lanePara2;
    lanePara2 = decideLanes("right", filteredLines, height, top_margin, right_lane_left_margin, right_lane_right_margin, bottom_margin);
    if (lanePara2.size()>0)
    {
        //Draw Right Lane on Image and intercept //Draw Intercept with right box on Image as blue circle
        //            line(outputImgToDrawOn, Point(lanePara2[0],lanePara2[1]), Point(lanePara2[2],lanePara2[3]), CV_RGB(0, 255, 0), 10, 8);
        //                    cv::circle(ou, Point(lanePara2[5] ,bottom_margin), 20, Scalar(255, 255, 0));
        rightThetaAndRow.push_back(Vec2f(lanePara2[6],lanePara2[4])); // push back right lane rho and theta;
        
    }
    return rightThetaAndRow ;
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*...................................................DETECT LEFT AND RIGHT LANE..................................................................*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Detect Lane Markings using Hough and narrow down so that it returns just left and right lane push back to vector can use scoreing or Ransac?
vector<Vec2f> detectLanes::HoughTransformReturnLeftRightLane(cv::Mat imgCanny){
    // intiialise lines vector and apply hough transforms
    vector<Vec2f> lines;
    cv::HoughLines( imgCanny, lines, 1, CV_PI/180, houghThreshold);
    // Filter out vector according to the angles of the left and right lane.
    vector<Vec2f> filteredLines(lines.size());
    auto it = copy_if(lines.begin(), lines.end(),
                      filteredLines.begin(),
                      [](const Vec2f &v) {
                          float angle = v[1];
                          return (angle <= 160*CV_PI/180 && angle >=100*CV_PI/180) || (angle>55*CV_PI/180 && angle<70*CV_PI/180) ;
                      });
    filteredLines.resize(std::distance(filteredLines.begin(),it));
    
    //Pull out the best left lane according to weighting algorithm
    vector<float> lanePara;
    vector<Vec2f> leftAndRightLaneThetaAndRow;
    
    lanePara = decideLanes("left", filteredLines, height, top_margin, left_lane_left_margin, left_lane_right_margin, bottom_margin);
    if (lanePara.size()>0)
    {   //Pull out the intercept
        //Draw Left Lane on Image and intercept Point
        //            cv::circle(outputImgToDrawOn, Point(lanePara[5] ,bottom_margin), 20, Scalar(255, 255, 0));
        //            line(outputImgToDrawOn, Point(lanePara[0],lanePara[1]), Point(lanePara[2],lanePara[3]), CV_RGB(0, 255, 0), 10, 8);
        leftAndRightLaneThetaAndRow.push_back(Vec2f(lanePara[6],lanePara[4])); // push back left lane rho and theta
        //            cout<<"laneLeft"<<lanePara[6]<<"  theta  = "<<lanePara[4]<<endl;
    }
    
    
    //Pull out the best right lane according to weighting algorithm
    vector<float> lanePara2;
    lanePara2 = decideLanes("right", filteredLines, height, top_margin, right_lane_left_margin, right_lane_right_margin, bottom_margin);
    if (lanePara2.size()>0)
    {
        //Draw Right Lane on Image and intercept //Draw Intercept with right box on Image as blue circle
        //            line(outputImgToDrawOn, Point(lanePara2[0],lanePara2[1]), Point(lanePara2[2],lanePara2[3]), CV_RGB(0, 255, 0), 10, 8);
        //            cv::circle(outputImgToDrawOn, Point(lanePara2[5] ,bottom_margin), 20, Scalar(255, 255, 0));
        leftAndRightLaneThetaAndRow.push_back(Vec2f(lanePara2[6],lanePara2[4])); // push back right lane rho and theta;
        
    }
    
    return leftAndRightLaneThetaAndRow;
}

void detectLanes:: drawBoundingBoxes(cv::Mat image){
    cv::circle(image, Point(left_lane_left_margin,top_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(left_lane_left_margin,bottom_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(left_lane_right_margin,top_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(left_lane_right_margin,bottom_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(right_lane_left_margin,top_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(right_lane_left_margin,bottom_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(right_lane_right_margin,top_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(right_lane_right_margin,bottom_margin), 20, Scalar(255, 255, 0));
}



vector<Vec2f> detectLanes::getLaneBeginningAndEndPoints(vector<Vec2f> leftandRightLineThetaAndRow){
    float rhoL = leftandRightLineThetaAndRow[0][0];
    float thetaL = leftandRightLineThetaAndRow[0][1];
    
    float rhoR = leftandRightLineThetaAndRow[1][0];
    float thetaR = leftandRightLineThetaAndRow[1][1];
    
    vector<Vec2f> endpoints;
    
    double aL = cos(thetaL), bL = sin(thetaL);
    double aR = cos(thetaR), bR = sin(thetaR);
    
    int LLBI = findIntercept(bottom_margin, rhoL, thetaL);
    int LLTI = (top_margin - (rhoL/bL))/(-aL/bL);
    
    endpoints.push_back(Vec2f(LLBI,bottom_margin));
    endpoints.push_back(Vec2f(LLTI,top_margin));
    
    int RLBI = findIntercept( bottom_margin, rhoR, thetaR);
    int RLTI = (top_margin - (rhoR/bR))/(-aR/bR);
    endpoints.push_back(Vec2f(RLBI,bottom_margin));
    endpoints.push_back(Vec2f(RLTI,top_margin));
    
    return endpoints;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*...................................................GET VANISHING POINT......................................................................*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Get Vanishing point from Lines
Point2f detectLanes::getVanishingPoint(vector<Vec2f> leftandRightLane){
    float rhoL = leftandRightLane[0][0]; // rho1
    float thetaL = leftandRightLane[0][1]; //theta1
    float rhoR = leftandRightLane[1][0]; // rho2
    float thetaR = leftandRightLane[1][1]; // theta2
    
    float aL = cos(thetaL), bL = sin(thetaL);
    float aR = cos(thetaR), bR = sin(thetaR);
    
    // calculate line intersection and compute vanishing point?
    // y = (-cos(theta)/sin(theta)*x + r/sin(theta))
    float x = ((rhoR/bR)-(rhoL/bL))/((aR/bR)-(aL/bL));
    float y = ((-aL/bL)*x) + (rhoL/bL);
    Point2f vanishingPoint(x,y);
    
    return vanishingPoint;
}
// Draw Lines on Image

vector<float> detectLanes::decideLanes(string laneDescriptor,vector<Vec2f> lines, int height,int topMargin,int leftMargin,int rightMargin, int bottomMargin)
{
    int distWeight = 2;
    int angleWeight = 10;
    int laneScore = -100000000;
    vector<float> lane;
    
    if (laneDescriptor.compare("left")==0){
        for(int i=0;i<lines.size();i++){
            float rho = lines[i][0];
            float theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            float intercept = findIntercept(bottom_margin, rho, theta);
            int topIntercept = (topMargin - (rho/b))/(-a/b);
            
            if (theta>0 && theta<M_PI/2 && intercept<rightMargin && topIntercept<rightMargin && intercept>leftMargin){
                float score = (-1*angleWeight*theta)+ (-1*distWeight * (rightMargin - intercept));
                if (score>laneScore){
                    laneScore=score;
                    Point2f LFBI(intercept, bottomMargin);
                    //                    int topIntercept = (topMargin - (rho/b))/(-a/b);
                    Point2f LFTI(topIntercept,topMargin);
                    float laneValues[7]= {LFBI.x,LFBI.y, LFTI.x,LFTI.y,theta,static_cast<float>(intercept),rho};
                    lane.insert(lane.end(), laneValues,laneValues+7);
                    
                }
            }
        }
    }
    
    if (laneDescriptor.compare("right")==0){
        for(int i=0;i<lines.size();i++){
            float rho = lines[i][0];
            float theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            
            int intercept = findIntercept(bottom_margin, rho, theta);
            int topIntercept = (topMargin - (rho/b))/(-a/b);
            
            if (theta< M_PI && theta>M_PI/2 && intercept<rightMargin  && topIntercept<rightMargin && intercept>leftMargin&& topIntercept>leftMargin){
                float score = (-1*angleWeight*theta)+ (-1*distWeight * (rightMargin - intercept));
                if (score>laneScore){
                    laneScore=score;
                    Point2f RLBI(intercept, bottomMargin);
                    Point2f RFTI(topIntercept,topMargin);
                    float laneValues[7]= {RLBI.x,RLBI.y, RFTI.x,RFTI.y,theta,static_cast<float>(intercept),rho};
                    lane.insert(lane.end(), laneValues,laneValues+7);
                }
            }
        }
    }
    return lane;
}


cv::Mat detectLanes::createTrapezoidROIInverse(cv::Mat TargetImg , vector<Vec2f> laneEndPoints, vector<int> IPMTopAndBotLimits)
{
    //    float height = TargetImg.rows;
    //    float width = TargetImg.cols;
    // account for 5% error
    
    Point LLBI(laneEndPoints[0][0]*1,laneEndPoints[0][1]);
    Point LLTI(laneEndPoints[1][0]*1,laneEndPoints[1][1]);
    Point RLBI(laneEndPoints[2][0]*1.5,laneEndPoints[2][1]);
    Point RLTI(laneEndPoints[3][0]*1.5,laneEndPoints[3][1]);
    
    
    //create Mask to Pull out ROI
    Mat mask = cv::Mat::zeros(TargetImg.size(),CV_8UC1);
    vector< vector<Point> >  co_ordinates;
    co_ordinates.push_back(vector<Point>());
    co_ordinates[0].push_back(LLBI);
    co_ordinates[0].push_back(LLTI);
    co_ordinates[0].push_back(RLTI);
    co_ordinates[0].push_back(RLBI);
    drawContours( mask,co_ordinates,0, Scalar(255),CV_FILLED, 8 );
    //    mask(Rect(0.1*width,0.3*height,0.55*width,0.5*height)) = 255;
    //    TargetImg &= mask;
    TargetImg -= mask;
    TargetImg = RectangleROISpecifyPoints(TargetImg,IPMTopAndBotLimits);
    
    return TargetImg;
}

cv::Mat detectLanes::RectangleROISpecifyPoints(cv::Mat TargetImg, vector<int> IPMTopAndBot)
{
    int width = TargetImg.cols;
    
    int ipmBottomMargin = IPMTopAndBot[0];
    int ipmTopMargin = IPMTopAndBot[1];
    
    Point LLB(0,ipmBottomMargin);
    Point LLT(0,ipmTopMargin);
    Point RLT(width,ipmTopMargin);
    Point RLB(width,ipmBottomMargin);
    
    
    //create Mask to Pull out ROI
    Mat mask = cv::Mat::zeros(TargetImg.size(),CV_8UC1);
    vector< vector<Point> >  co_ordinates;
    co_ordinates.push_back(vector<Point>());
    co_ordinates[0].push_back(LLB);
    co_ordinates[0].push_back(LLT);
    co_ordinates[0].push_back(RLT);
    co_ordinates[0].push_back(RLB);
    
    drawContours( mask,co_ordinates,0, Scalar(255),CV_FILLED, 8 );
    
    TargetImg &= mask;
    //    imshow("Masked Image",TargetImg);
    return TargetImg;
}


float detectLanes::findIntercept(int bottomMargin,float rho,float theta)
{
    float bottomIntercept;
    double a = cos(theta), b = sin(theta);
    bottomIntercept = (bottomMargin - (rho/b))/(-a/b);
    return bottomIntercept;
}

void detectLanes::drawLeftLinesOnImage(cv::Mat &image, vector<Vec2f> lines){
    float rho1 = lines[0][0];
    float theta1 = lines[0][1];
    double a1 = cos(theta1), b1 = sin(theta1);
    int botIntercept1 = findIntercept(bottom_margin, rho1, theta1);
    int topIntercept1 = (top_margin - (rho1/b1))/(-a1/b1);
    Point2f LTI(topIntercept1,top_margin);
    Point2f LBI(botIntercept1,bottom_margin);
    line(image, LBI, LTI, CV_RGB(0, 255, 0), 10, 8);
}

void detectLanes::drawRightLinesOnImage(cv::Mat &image, vector<Vec2f> lines){
    float rho2 = lines[0][0];
    float theta2 = lines[0][1];
    double a2 = cos(theta2), b2 = sin(theta2);
    int botIntercept2 = findIntercept(bottom_margin, rho2, theta2);
    int topIntercept2 = (top_margin - (rho2/b2))/(-a2/b2);
    Point2f RTI(topIntercept2,top_margin);
    Point2f RBI(botIntercept2,bottom_margin);
    line(image, RTI, RBI, CV_RGB(255, 0, 0), 5, 8);
    
}


void detectLanes::drawFinalLinesOnImage(cv::Mat image, vector<Vec2f> lines){
    float rho1 = lines[0][0];
    float theta1 = lines[0][1];
    double a1 = cos(theta1), b1 = sin(theta1);
    int intercept1 = findIntercept(bottom_margin, rho1, theta1);
    int topIntercept1 = (top_margin - (rho1/b1))/(-a1/b1);
    Point2f LTI(topIntercept1,top_margin);
    Point2f LBI(intercept1,bottom_margin);
    line(image, LBI, LTI, CV_RGB(0, 255, 0), 10, 8);
    
    
    if(lines.size()>1){
        float rho2 = lines[1][0];
        float theta2 = lines[1][1];
        double a2 = cos(theta2), b2 = sin(theta2);
        int intercept2 = findIntercept( bottom_margin, rho2, theta2);
        int topIntercept2 = (top_margin - (rho2/b2))/(-a2/b2);
        Point2f RTI(topIntercept2,top_margin);
        Point2f RBI(intercept2,bottom_margin);
        line(image, RTI, RBI, CV_RGB(0, 255, 0), 10, 8);
    }
}



cv::Mat detectLanes::createRectangleROI(cv::Mat TargetImg)
{
    float height = TargetImg.rows;
    float width = TargetImg.cols;
    //        cv::Rect roi(0,420,1000,600);
    //create Mask to Pull out ROI
    Mat mask = cv::Mat::zeros(TargetImg.size(),CV_8UC1);
    
    mask(Rect(0.1*width,0.3*height,0.8*width,0.5*height)) = 255;
    TargetImg &= mask;
    //    imshow("Masked Image",TargetImg);
    return TargetImg;
}

//Create Mask to extract region of interest maybe better to extract a better mask for  roads
cv::Mat detectLanes::createTrapezoidROI(cv::Mat TargetImg)
{
    float height = TargetImg.rows;
    float width = TargetImg.cols;
    
    //create Mask to Pull out ROI
    Mat mask = cv::Mat::zeros(TargetImg.size(),CV_8UC1);
    vector< vector<Point> >  co_ordinates;
    co_ordinates.push_back(vector<Point>());
    co_ordinates[0].push_back(Point(0,height));
    co_ordinates[0].push_back(Point(0,height*5/6));
    co_ordinates[0].push_back(Point(width/3,height/2.5));
    co_ordinates[0].push_back(Point(2*width/3,height/2.5));
    co_ordinates[0].push_back(Point(width,height*5/6));
    co_ordinates[0].push_back(Point(width,height));
    drawContours( mask,co_ordinates,0, Scalar(255),CV_FILLED, 8 );
    
    //    mask(Rect(0.1*width,0.3*height,0.55*width,0.5*height)) = 255;
    TargetImg &= mask;
    //    imshow("Masked Image",TargetImg);
    return TargetImg;
}


