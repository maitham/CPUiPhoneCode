//
//  OpticFlow.cpp
//  LaneCuttingAversion
//
//  Created by Maitham Dib on 18/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#include "OpticFlow.hpp"
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;


static const double pi = 3.14159265358979323846;

inline static double square(int a)
{
    return a * a;
}


cv::Mat drawOpticFlowArrows(int numberOfFeatures, vector<unsigned char> opticalFlowFoundFeature, vector<Point2f> frame1Features ,vector<Point2f> frame2Features,cv::Mat frame1){
    int obstacleCount=0;
    /* For fun (and debugging :)), let's draw the flow field. */
    for(int i = 0; i < numberOfFeatures; i++)
    {
        /* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
        if ( opticalFlowFoundFeature[i] == 0 )	continue;
        int line_thickness;
        line_thickness = 1;
        /* CV_RGB(red, green, blue) is the red, green, and blue components
         * of the color you want, each out of 255.*/
        CvScalar line_color;
        line_color = CV_RGB(255,0,0);
        
        /* Let's make the flow field look nice with arrows. *
         * The arrows will be a bit too short for a nice visualization because of the high framerate
         * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
         */
        CvPoint p,q;
        p.x = (int) frame1Features[i].x;
        p.y = (int) frame1Features[i].y;
        q.x = (int) frame2Features[i].x;
        q.y = (int) frame2Features[i].y;
        
        double angle;
        angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
        //        cout<<angle*180/M_PI<<endl;
        double hypotenuse;
        hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );
        
        /* Here we lengthen the arrow by a factor of three. */
        q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
        q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
        
        if(angle<=-90*M_PI/180&&angle>=-180*M_PI/180){
            
            obstacleCount++;
            line(frame1, p, q, line_color, line_thickness, CV_AA, 0 );
            
            /* Now draw the tips of the arrow.  I do some scaling so that the
             * tips look proportional to the main line of the arrow.
             */
            p.x = (int) (q.x + 9 * cos(angle + pi / 4));
            p.y = (int) (q.y + 9 * sin(angle + pi / 4));
            line( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
            p.x = (int) (q.x + 9 * cos(angle - pi / 4));
            p.y = (int) (q.y + 9 * sin(angle - pi / 4));
            line( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
        }
        /* Now we draw the main line of the arrow. */
        /* "frame1" is the frame to draw on.
         * "p" is the point where the line begins.
         * "q" is the point where the line stops.
         * "CV_AA" means antialiased drawing.
         * "0" means no fractional bits in the center cooridinate or radius.
         */
        
    }
    /* Now display the image we drew on.  Recall that "Optical Flow" is the name of
     * the window we created above.
     */
    char str[200];
    sprintf(str,"%d  Lane Encroachments",obstacleCount);
    putText(frame1, str, Point2f(30,30), FONT_HERSHEY_COMPLEX_SMALL, 2,  Scalar(0,0,255,255));
    
    return frame1;
}

cv::Mat calcOpticFlowAndDraw(cv::Mat frame1_1C,cv::Mat frame2_1C, cv::Mat frame1 ){
    /* Shi and Tomasi Feature Tracking! */
    int numberOfFeatures =400;
    
    vector<Point2f> frame1Features;
    // Find GOod Features to Track
    goodFeaturesToTrack(frame1_1C, frame1Features, numberOfFeatures, .01, .01);
    
    /* Pyramidal Lucas Kanade Optical Flow! */
    vector<Point2f> frame2Features;
    vector<unsigned char> opticalFlowFoundFeature;
    vector<float> opticalFlowError;
    // This termination criteria tells the algorithm to stop when it has either done 20 iterations or when epsilon is better than .3.
    TermCriteria optical_flow_termination_criteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );
    
    calcOpticalFlowPyrLK(frame1_1C, frame2_1C, frame1Features, frame2Features, opticalFlowFoundFeature, opticalFlowError);
    
    frame1 = drawOpticFlowArrows(numberOfFeatures, opticalFlowFoundFeature, frame1Features, frame2Features,frame1);
    
    return frame1;
}

//
float getLineMagnitude(Vec4f interestingPoints){
    float x1 = interestingPoints[0];
    float y1 = interestingPoints[1];
    float x2 = interestingPoints[2];
    float y2 = interestingPoints[3];
    float x = x2-x1;
    float y = y2-y1;
    
    
    float resultantDistance = sqrt((x*x)+(y*y));
    return resultantDistance;
}

float getAngle(Vec4f interestingPoints){
    float x1 = interestingPoints[0];
    float y1 = interestingPoints[1];
    float x2 = interestingPoints[2];
    float y2 = interestingPoints[3];
    
    float x = abs(x2) - abs(x1);
    float y = abs(y2) - abs(y1);
    
    float angle =atan2f(y, 2);
    
    return angle;
}


bool isCarIntercepting(vector<Vec4f> interestingPoints){
    bool isCarCuttingIn;
    
    //Find angles of Line and Size and store
    for (int i =0; i<interestingPoints.size(); i++) {
        float lineMagnitude= getLineMagnitude(interestingPoints[i]);
        float lineAngle= getAngle(interestingPoints[i]);
    }
    
    
    return isCarCuttingIn;
}



cv::Mat getDenseOpticFlow(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn){
    Mat flow, frame;
    // some faster than mat image container
    UMat  flowUmat, prevgray;
    vector<Vec4f> interestingPoints;
    vector<Vec4f> nonInterestingPoints;
    vector<Vec4f> allPoints;
    vector<float> angles;
    vector<float> absoluteSize;
    
    // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous and current frame
    
    // calculate optical flow
    calcOpticalFlowFarneback(frame1_1C, frame2_1C, flowUmat, 0.5, 2, 50, 3, 5, 1.1, 0);
    // copy Umat container to standard Mat
    flowUmat.copyTo(flow);
    
    // By y += 5, x += 5 you can specify the grid
    for (int y = 0; y < imageToDrawOn.rows; y += 30){
        for (int x = 0; x < imageToDrawOn.cols; x += 30)
        {
            // get the flow from y, x position * 10 for better visibility
            const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
            line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(0,0,255));
            allPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
            
            // draw line at flow direction
            int minimumDistance = 2;
            //Minimum Parameters for angle and resultant distance
            double resultantDistance = sqrt((flowatxy.x*flowatxy.x)+(flowatxy.y*flowatxy.y));
            float angletemp = atanf((abs(flowatxy.y))/(abs(flowatxy.x)));
            //                            cout<< "angletemp= "<<angletemp*180/M_PI<<endl;
            float calculatedAngle;
            if(flowatxy.x<0 && flowatxy.y<0 ){
                calculatedAngle = M_PI-angletemp;
            }else if (flowatxy.x<0 && flowatxy.y>0){
                calculatedAngle =M_PI + angletemp;
            }else if(flowatxy.x>0&&flowatxy.y>0 ){
                calculatedAngle = 2*M_PI - angletemp;
            }else{
                calculatedAngle = angletemp;
            }
            //Filter Lines
            if (resultantDistance>minimumDistance){
                if(calculatedAngle > 160*M_PI/180 && calculatedAngle <270*M_PI/180){
                    angles.push_back(calculatedAngle);
                    absoluteSize.push_back(resultantDistance);
                    interestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
                    //                                    cout<<"calculatedAngle= "<<calculatedAngle*180/M_PI<<endl;
                    //                                    imshow("imageDebug", imageToDrawOn);
                    //
                    //                                    waitKey();
                }else{
                    nonInterestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
                }
            }
            circle(imageToDrawOn, Point(x, y), 1, Scalar(0, 0, 0), -1);
        }
    }
    
    bool isCarCuttingIn;
    int sizeOfInterestingPoints = interestingPoints.size();
    int sizeOfNonInterestingPoints = nonInterestingPoints.size();
    
    float sumAngles;
    float sumDistances;
    
    //    cout<<"Interesting Points"<<interestingPoints.size()<<endl;
    //    cout<<"Non-Interesting Points"<<nonInterestingPoints.size()<<endl;
    
    if(interestingPoints.size()>nonInterestingPoints.size()){
        //average the angles and the magnitude
        for (int k=0; k<angles.size(); k++) {
            sumAngles += angles[k];
            sumDistances += absoluteSize[k];
        }
        float averageAngle =sumAngles/angles.size();
        float averageDistances = sumDistances/absoluteSize.size();
        
        float width  = imageToDrawOn.cols;
        float height = imageToDrawOn.rows;
        
        float averageY;
        float averageX;
        
        if(averageAngle>90*M_PI/180 && averageAngle<180*M_PI/180){
            // x -ve y +ve
            averageAngle = averageAngle-(90*M_PI/180);
            
            averageY = averageDistances * -sin(averageAngle);
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>180*M_PI/180 && averageAngle<270*M_PI/180){
            // x -ve y -ve
            averageAngle = averageAngle-(180*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>270*M_PI/180 && averageAngle<360*M_PI/180){
            // x +ve y -ve
            averageAngle = averageAngle-(270*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * (cos(averageAngle));
        }else{
            averageX = averageDistances*cos(averageAngle);
            averageY = averageDistances*sin(averageAngle);
        }
        
        
        //        float y = averageDistances*sin(averageAngle);
        //        float x = averageDistances*cos(averageAngle);
        
        //Draw Large arrows
        arrowedLine(imageToDrawOn, Point(width/2,height/2), Point(averageX+width/2,averageY+height/2), Scalar(255,0,0));
        // turn bool on cutting lane
        isCarCuttingIn = true;
        //        cout<<"Car cutting in"<<endl;
    }else{
        //turn Bool on cutting lane
        //        cout<<"Car NOT cutting in"<<endl;
        isCarCuttingIn = false;
    }
    
    if (isCarCuttingIn) {
        circle(imageToDrawOn, Point(100,50), 10, Scalar(0,0,255),-1, 8, 0);
        //        circle(imageToDrawOn, Point(100,50), 10, Scalar(255,0,0));
    }else{
        circle(imageToDrawOn, Point(100,50), 10, Scalar(255,255,255),-1, 8, 0);
    }
    
    return imageToDrawOn;
}



cv::Mat getDenseOpticFlowRobustRight(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn){
    Mat flow, frame;
    // some faster than mat image container
    UMat  flowUmat, prevgray;
    vector<Vec4f> interestingPoints;
    vector<Vec4f> nonInterestingPoints;
    vector<Vec4f> allPoints;
    vector<float> angles;
    vector<float> absoluteSize;
    
    // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous and current frame
    
    // calculate optical flow
    calcOpticalFlowFarneback(frame1_1C, frame2_1C, flowUmat, 0.5, 2, 50, 3, 5, 1.1, 0);
    // copy Umat container to standard Mat
    flowUmat.copyTo(flow);
    
    // By y += 5, x += 5 you can specify the grid
    for (int y = 0; y < imageToDrawOn.rows; y += 20){
        for (int x = 0; x < imageToDrawOn.cols; x += 20)
        {
            // get the flow from y, x position * 10 for better visibility
            const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
            line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(0,0,255));
            allPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
            
            // draw line at flow direction
            int minimumDistance = 10;
            //Minimum Parameters for angle and resultant distance
            double resultantDistance = sqrt((flowatxy.x*flowatxy.x)+(flowatxy.y*flowatxy.y));
            
            
            float angletemp = atanf((abs(flowatxy.y))/(abs(flowatxy.x)));
            //                            cout<< "angletemp= "<<angletemp*180/M_PI<<endl;
            float calculatedAngle;
            if(flowatxy.x<0 && flowatxy.y<0 ){
                calculatedAngle = M_PI-angletemp;
            }else if (flowatxy.x<0 && flowatxy.y>0){
                calculatedAngle =M_PI + angletemp;
            }else if(flowatxy.x>0&&flowatxy.y>0 ){
                calculatedAngle = 2*M_PI - angletemp;
            }else{
                calculatedAngle = angletemp;
            }
            //Filter Lines
            if (resultantDistance>minimumDistance){
                if(calculatedAngle > 135*M_PI/180 && calculatedAngle<260*M_PI/180){
                    angles.push_back(calculatedAngle);
                    absoluteSize.push_back(resultantDistance);
                    interestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
                    //                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
                    //                                    cout<<"calculatedAngle= "<<calculatedAngle*180/M_PI<<endl;
                    //                                    imshow("imageDebug", imageToDrawOn);
                    //
                    //                                    waitKey();
                }else{
                    nonInterestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
                }
            }
            //            if(resultantDistance/10> 6){
            //                line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,255));
            //            }
            circle(imageToDrawOn, Point(x, y), 1, Scalar(0, 0, 0), -1);
        }
    }
    
    bool isCarCuttingIn;
    int sizeOfInterestingPoints = interestingPoints.size();
    int sizeOfNonInterestingPoints = nonInterestingPoints.size();
    
    float sumAngles;
    float sumDistances;
    
    //    cout<<"Interesting Points"<<interestingPoints.size()<<endl;
    //    cout<<"Non-Interesting Points"<<nonInterestingPoints.size()<<endl;
    
    if(sizeOfInterestingPoints>sizeOfNonInterestingPoints){
        //average the angles and the magnitude
        for (int k=0; k<angles.size(); k++) {
            sumAngles += angles[k];
            sumDistances += absoluteSize[k];
        }
        float averageAngle =sumAngles/angles.size();
        float averageDistances = sumDistances/absoluteSize.size();
        
        float width  = imageToDrawOn.cols;
        float height = imageToDrawOn.rows;
        
        float averageY;
        float averageX;
        
        if(averageAngle>90*M_PI/180 && averageAngle<180*M_PI/180){
            // x -ve y +ve
            averageAngle = averageAngle-(90*M_PI/180);
            
            averageY = averageDistances * -sin(averageAngle);
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>180*M_PI/180 && averageAngle<270*M_PI/180){
            // x -ve y -ve
            averageAngle = averageAngle-(180*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>270*M_PI/180 && averageAngle<360*M_PI/180){
            // x +ve y -ve
            averageAngle = averageAngle-(270*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * (cos(averageAngle));
        }else{
            averageX = averageDistances*cos(averageAngle);
            averageY = averageDistances*sin(averageAngle);
        }
        
        
        //        float y = averageDistances*sin(averageAngle);
        //        float x = averageDistances*cos(averageAngle);
        
        //Draw Large arrows
        arrowedLine(imageToDrawOn, Point(width/2,height/2), Point(averageX+width/2,averageY+height/2), Scalar(255,0,0));
        // turn bool on cutting lane
        isCarCuttingIn = true;
        //        cout<<"Car cutting in"<<endl;
    }else{
        //turn Bool on cutting lane
        //        cout<<"Car NOT cutting in"<<endl;
        isCarCuttingIn = false;
    }
    
    if (isCarCuttingIn) {
        circle(imageToDrawOn, Point(100,50), 10, Scalar(0,0,255),-1, 8, 0);
//        objectCuttingIn=true;
    }else{
        circle(imageToDrawOn, Point(100,50), 10, Scalar(255,255,255),-1, 8, 0);
//        objectCuttingIn=false;
        
    }
    
    return imageToDrawOn;
}

cv::Mat getDenseOpticFlowRobustLeft(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn){
    Mat flow, frame;
    // some faster than mat image container
    UMat  flowUmat, prevgray;
    vector<Vec4f> interestingPoints;
    vector<Vec4f> nonInterestingPoints;
    vector<Vec4f> allPoints;
    vector<float> angles;
    vector<float> absoluteSize;
    
    // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous and current frame
    
    // calculate optical flow
    calcOpticalFlowFarneback(frame1_1C, frame2_1C, flowUmat, 0.5, 2, 50, 3, 5, 1.1, 0);
    // copy Umat container to standard Mat
    flowUmat.copyTo(flow);
    
    // By y += 5, x += 5 you can specify the grid
    for (int y = 0; y < imageToDrawOn.rows; y += 20){
        for (int x = 0; x < imageToDrawOn.cols; x += 20)
        {
            // get the flow from y, x position * 10 for better visibility
            const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
            line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(0,0,255));
            allPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
            
            // draw line at flow direction
            int minimumDistance = 10;
            //Minimum Parameters for angle and resultant distance
            double resultantDistance = sqrt((flowatxy.x*flowatxy.x)+(flowatxy.y*flowatxy.y));
            float angletemp = atanf((abs(flowatxy.y))/(abs(flowatxy.x)));
            //                            cout<< "angletemp= "<<angletemp*180/M_PI<<endl;
            float calculatedAngle;
            if(flowatxy.x<0 && flowatxy.y<0 ){
                calculatedAngle = M_PI-angletemp;
            }else if (flowatxy.x<0 && flowatxy.y>0){
                calculatedAngle =M_PI + angletemp;
            }else if(flowatxy.x>0&&flowatxy.y>0 ){
                calculatedAngle = 2*M_PI - angletemp;
            }else{
                calculatedAngle = angletemp;
            }
            //Filter Lines
            if (resultantDistance>minimumDistance){
                if(calculatedAngle <20*M_PI/180 || calculatedAngle>270*M_PI/180){
                    angles.push_back(calculatedAngle);
                    absoluteSize.push_back(resultantDistance);
                    interestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
                    //                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
                    //                                    cout<<"calculatedAngle= "<<calculatedAngle*180/M_PI<<endl;
                    //                                    imshow("imageDebug", imageToDrawOn);
                    //
                    //                                    waitKey();
                }else{
                    nonInterestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
                }
            }
            circle(imageToDrawOn, Point(x, y), 1, Scalar(0, 0, 0), -1);
        }
    }
    
    bool isCarCuttingIn;
    int sizeOfInterestingPoints = interestingPoints.size();
    int sizeOfNonInterestingPoints = nonInterestingPoints.size();
    
    float sumAngles;
    float sumDistances;
    
    //    cout<<"Interesting Points"<<interestingPoints.size()<<endl;
    //    cout<<"Non-Interesting Points"<<nonInterestingPoints.size()<<endl;
    
    if(interestingPoints.size()>nonInterestingPoints.size()){
        //average the angles and the magnitude
        for (int k=0; k<angles.size(); k++) {
            sumAngles += angles[k];
            sumDistances += absoluteSize[k];
        }
        float averageAngle =sumAngles/angles.size();
        float averageDistances = sumDistances/absoluteSize.size();
        
        float width  = imageToDrawOn.cols;
        float height = imageToDrawOn.rows;
        
        float averageY;
        float averageX;
        
        if(averageAngle>90*M_PI/180 && averageAngle<180*M_PI/180){
            // x -ve y +ve
            averageAngle = averageAngle-(90*M_PI/180);
            
            averageY = averageDistances * -sin(averageAngle);
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>180*M_PI/180 && averageAngle<270*M_PI/180){
            // x -ve y -ve
            averageAngle = averageAngle-(180*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>270*M_PI/180 && averageAngle<360*M_PI/180){
            // x +ve y -ve
            averageAngle = averageAngle-(270*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * (cos(averageAngle));
        }else{
            averageX = averageDistances*cos(averageAngle);
            averageY = averageDistances*sin(averageAngle);
        }
        
        
        //        float y = averageDistances*sin(averageAngle);
        //        float x = averageDistances*cos(averageAngle);
        
        //Draw Large arrows
        arrowedLine(imageToDrawOn, Point(width/2,height/2), Point(averageX+width/2,averageY+height/2), Scalar(255,0,0));
        // turn bool on cutting lane
        isCarCuttingIn = true;
        //        cout<<"Car cutting in"<<endl;
    }else{
        //turn Bool on cutting lane
        //        cout<<"Car NOT cutting in"<<endl;
        isCarCuttingIn = false;
    }
    
    if (isCarCuttingIn) {
        circle(imageToDrawOn, Point(100,50), 10, Scalar(0,0,255),-1, 8, 0);
//        objectCuttingIn=true;
    }else{
        circle(imageToDrawOn, Point(100,50), 10, Scalar(255,255,255),-1, 8, 0);
//        objectCuttingIn=false;
    }
    
    return imageToDrawOn;
}