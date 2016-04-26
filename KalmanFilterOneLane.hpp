//
//  CKalmanFilter.hpp
//  HistogramSegmentation
//
//  Created by Maitham Dib on 16/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

class KalmanFilterOneLane
{
public:
    KalmanFilterOneLane(vector<Vec2f>);
    ~KalmanFilterOneLane();
    vector<Vec2f> predictOneLane();
    vector<Vec2f> updateOneLane(vector<Vec2f>);
    
    KalmanFilter* kalmanOneLane;
    vector<Vec2f> prevResultOneLane;
    
};