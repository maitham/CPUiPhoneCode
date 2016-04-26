#include <iostream>
#include "KalmanFilterOneLane.hpp"

using namespace cv;
using namespace std;

// Constructor
KalmanFilterOneLane::KalmanFilterOneLane(vector<Vec2f> p){
    
    kalmanOneLane = new KalmanFilter( 2, 2, 0 ); // 4 measurement and state parameters
    kalmanOneLane->transitionMatrix = (Mat_<float>(2, 2) << 1,0, 0,1);
    
    // Initialization
    prevResultOneLane = p;
    kalmanOneLane->statePre.at<float>(0) = p[0][0]; // r1
    kalmanOneLane->statePre.at<float>(1) = p[0][1]; // theta1
//    kalman->statePre.at<float>(2) = p[1][0]; // r2
//    kalman->statePre.at<float>(3) = p[1][1]; // theta2
    
    kalmanOneLane->statePost.at<float>(0)=p[0][0];
    kalmanOneLane->statePost.at<float>(1)=p[0][1];
//    kalman->statePost.at<float>(2)=p[1][0];
//    kalman->statePost.at<float>(3)=p[1][1];
    
    setIdentity(kalmanOneLane->measurementMatrix);
    setIdentity(kalmanOneLane->processNoiseCov, Scalar::all(1e-4));
    setIdentity(kalmanOneLane->measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(kalmanOneLane->errorCovPost, Scalar::all(5));
}

// Destructor
KalmanFilterOneLane::~KalmanFilterOneLane(){
    delete kalmanOneLane;
}

// Prediction
vector<Vec2f> KalmanFilterOneLane::predictOneLane(){
    Mat prediction = kalmanOneLane->predict(); // predict the state of the next frame
    prevResultOneLane[0][0] = prediction.at<float>(0);prevResultOneLane[0][1] = prediction.at<float>(1);
//    prevResult[1][0] = prediction.at<float>(2);prevResult[1][1] = prediction.at<float>(3);
    return prevResultOneLane;
    
}

// Correct the prediction based on the measurement
vector<Vec2f> KalmanFilterOneLane::updateOneLane(vector<Vec2f> measure){
    
    
    Mat_<float> measurement(2,1);
    measurement.setTo(Scalar(0));
    
    measurement.at<float>(0) = measure[0][0];measurement.at<float>(1) = measure[0][1];
//    measurement.at<float>(2) = measure[1][0];measurement.at<float>(3) = measure[1][1];
    
    Mat estimated = kalmanOneLane->correct(measurement); // Correct the state of the next frame after obtaining the measurements
    
    // Update the measurement
    if(estimated.at<float>(0) < estimated.at<float>(2)){
        measure[0][0] = estimated.at<float>(0);measure[0][1] = estimated.at<float>(1);
//        measure[1][0] = estimated.at<float>(2);measure[1][1] = estimated.at<float>(3);
    }
    else{
        measure[0][0] = estimated.at<float>(2);measure[0][1] = estimated.at<float>(3);
//        measure[1][0] = estimated.at<float>(0);measure[1][1] = estimated.at<float>(1);
    }
        
    return measure; // return the measurement
    
}