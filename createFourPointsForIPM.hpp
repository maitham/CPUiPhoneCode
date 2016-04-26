//
//  createFourPointsForIPM.hpp
//  HistogramSegmentation
//
//  Created by Maitham Dib on 02/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#ifndef createFourPointsForIPM_hpp
#define createFourPointsForIPM_hpp

#include <stdio.h>
#include "matrixMethods.hpp"

struct trapeziumCoordinates{
    Matrix point1,point2,point3,point4;
};

trapeziumCoordinates createFourPointsForIPM(const float vanishingPointXCoordinate, const float vanishingPointYCoordinate );
trapeziumCoordinates createFourPointsForIPMLeftLane(const float vanishingPointXCoordinate, const float vanishingPointYCoordinate);
trapeziumCoordinates createFourPointsForIPMRightLane(const float vanishingPointXCoordinate, const float vanishingPointYCoordinate);

struct cameraInfo{
    float imageWidth_M;
    float imageHeight_N;
    double verticalAperture_fy;
    double horizontalAperture_fx;
    double principalPointX_u;
    double principalPointY_v;
    float cameraHeight;
    float pitchAngle;
    float yawAngle;
    float rollAngle;
    float imageWidth;
    float imageHeight;
};
#endif /* createFourPointsForIPM_hpp */
