//
//  cameraCalibration.swift
//  ObjectDetectionSwift
//
//  Created by Maitham Dib on 27/02/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

import Foundation
import GLKit
import AVFoundation

class cameraCalibration{
    
    class func createCameraProjection(captureDevice:AVCaptureDevice!, output:UIImageView, zNear:Float, zFar:Float)->[[Float]]{
   
//        var extrinsicMatrix = Array(count:4, repeatedValue:Array(count:4, repeatedValue:Float()))
//        var mout = Array(count:4, repeatedValue:Array(count:4, repeatedValue:Float()))
//        var intrinsicMatrix = Array(count:3, repeatedValue:Array(count:3, repeatedValue:Float()))
//        var rotationMatrix = Array(count:4, repeatedValue:Array(count:4, repeatedValue:Float()))
//        var translationMatrix = Array(count:4, repeatedValue:Array(count:4, repeatedValue:Float()))
////        var mout = Array(count:4, repeatedValue:Array(count:4, repeatedValue:Float()))
//        let frameSize = output.frame
//
//        // Create Intrinsic Matrix
//        let cx = Float(frameSize.width/2)
//        let cy = Float(frameSize.height/2)
//        let HFOV = Float(captureDevice.activeFormat.videoFieldOfView)
//        let VFOV = Float((HFOV)/cx)*cy
//        
//        let fx = abs(Float(frameSize.width) / (2 * tan(HFOV / 180 * Float(M_PI) / 2)))
//        let fy = abs(Float(frameSize.height) / (2 * tan(VFOV / 180 * Float(M_PI) / 2)))
//        intrinsicMatrix[0][0] = fx;
//        intrinsicMatrix[0][1] = 0.0;
//        intrinsicMatrix[0][2] = cx;
//        
//        intrinsicMatrix[1][0] = 0;
//        intrinsicMatrix[1][1] = fy;
//        intrinsicMatrix[1][2] = cy;
//        
//        intrinsicMatrix[2][0] = 0.0;
//        intrinsicMatrix[2][1] = 0.0;
//        intrinsicMatrix[2][2] = 1.0;
//
//        // Create Camera Extrinsic Matrix
//        let cameraAngleinRadians = GLKMathDegreesToRadians(20)
//        let heightOfCameraFromGround:Float = 0.7
//            //Create Rotation Matrix
//            rotationMatrix[0][0] = 1;
//            rotationMatrix[0][1] = 0.0;
//            rotationMatrix[0][2] = 0.0;
//            rotationMatrix[0][3] = 0.0;
//        
//            rotationMatrix[1][0] = 0.0;
//            rotationMatrix[1][1] = cos(cameraAngleinRadians);
//            rotationMatrix[1][2] = -sin(cameraAngleinRadians);
//            rotationMatrix[1][3] = 0.0;
//        
//            rotationMatrix[2][0] = 0.0;
//            rotationMatrix[2][1] = sin(cameraAngleinRadians);
//            rotationMatrix[2][2] = cos(cameraAngleinRadians)
//            rotationMatrix[2][3] = 0.0;
//        
//            rotationMatrix[3][0] = 0.0;
//            rotationMatrix[3][1] = 0.0;
//            rotationMatrix[3][2] = 0.0;
//            rotationMatrix[3][3] = 1.0;
//
//            //Create Translation Matrix
//            translationMatrix[0][0] = 1;
//            translationMatrix[0][1] = 0.0;
//            translationMatrix[0][2] = 0.0;
//            translationMatrix[0][3] = 0.0;
//        
//            translationMatrix[1][0] = 0.0;
//            translationMatrix[1][1] = 1.0;
//            translationMatrix[1][2] = 0.0;
//            translationMatrix[1][3] = 0.0;
//        
//            translationMatrix[2][0] = 0.0;
//            translationMatrix[2][1] = 0.0;
//            translationMatrix[2][2] = 1.0
//            translationMatrix[2][3] = -heightOfCameraFromGround/sinf(cameraAngleinRadians);
//        
//            translationMatrix[3][0] = 0.0;
//            translationMatrix[3][1] = 0.0;
//            translationMatrix[3][2] = 0.0;
//            translationMatrix[3][3] = 1.0;
//
        var intrinsicMatrix = Array(count:3, repeatedValue:Array(count:3, repeatedValue:Float()))
        
        let frameSize = output.frame
        let cx = Float(frameSize.width/2)
        let cy = Float(frameSize.height/2)
        let HFOV = Float(captureDevice.activeFormat.videoFieldOfView)
        let VFOV = Float((HFOV)/cx)*cy
        
        let fx = abs(Float(frameSize.width) / (2 * tan(HFOV / 180 * Float(M_PI) / 2)))
        let fy = abs(Float(frameSize.height) / (2 * tan(VFOV / 180 * Float(M_PI) / 2)))
        
        intrinsicMatrix[0][0] = fx;
        intrinsicMatrix[0][1] = 0.0;
        intrinsicMatrix[0][2] = cx;
        
        intrinsicMatrix[1][0] = 0;
        intrinsicMatrix[1][1] = fy;
        intrinsicMatrix[1][2] = cy;
        
        intrinsicMatrix[2][0] = 0.0;
        intrinsicMatrix[2][1] = 0.0;
        intrinsicMatrix[2][2] = 1.0;

        
        
//        let frameSize = output.frame
//        var mout = Array(count:4, repeatedValue:Array(count:4, repeatedValue:Float()))
//        let aspect = Float(frameSize.width/frameSize.height)
//        let fieldOfView = GLKMathRadiansToDegrees(captureDevice.activeFormat.videoFieldOfView)
//        
//        let f = 1.0 / tanf(fieldOfView/2.0);
//        
//        mout[0][0] = (f / aspect);
//        mout[0][1] = 0.0;
//        mout[0][2] = 0.0;
//        mout[0][3] = 0.0;
//        
//        mout[1][0] = 0.0;
//        mout[1][1] = (f);
//        mout[1][2] = 0.0;
//        mout[1][3] = 0.0;
//        
//        mout[2][0] = 0.0;
//        mout[2][1] = 0.0;
//        mout[2][2] = ((zFar+zNear) / (zNear-zFar));
//        mout[2][3] = -1.0;
//        
//        mout[3][0] = 0.0;
//        mout[3][1] = 0.0;
//        mout[3][2] = (2 * zFar * zNear /  (zNear-zFar));
//        mout[3][3] = 0.0
        
        return intrinsicMatrix  
    }

}
