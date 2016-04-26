//
//  CameraUtil.swift
//  ObjectDetectionSwift
//
//  Created by Maitham Dib on 22/02/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

import Foundation
import UIKit
import AVFoundation

//** Taken from github reference

class CameraUtil {
    
    // sampleBuffer- A CMSampleBuffer is a Core Foundation object containing zero or more compressed (or uncompressed) samples of a particular media type
    class func imageFromSampleBuffer(sampleBuffer: CMSampleBufferRef) -> UIImage {
        let imageBuffer: CVImageBufferRef! = CMSampleBufferGetImageBuffer(sampleBuffer)
        
        // Locks the base address of the pixel buffer
        CVPixelBufferLockBaseAddress(imageBuffer, 0)
        
        //Returns the base address of the plane at the specified plane index.
        //The base address of the plane, or NULL for nonplanar pixel buffers.
        let baseAddress: UnsafeMutablePointer<Void> = CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 0)
        
        // Get Size of Image Buffer
        let bytesPerRow: Int = CVPixelBufferGetBytesPerRow(imageBuffer)
        let width: Int = CVPixelBufferGetWidth(imageBuffer)
        let height: Int = CVPixelBufferGetHeight(imageBuffer)
        
        // Creates a device-dependent RGB color space.
        // A device-dependent RGB color space. You are responsible for releasing this object by calling CGColorSpaceRelease. If unsuccessful, returns NULL.
        let colorSpace: CGColorSpaceRef! = CGColorSpaceCreateDeviceRGB()
        
        // Bitmap graphic context
        let bitsPerCompornent: Int = 8
        let bitmapInfo = CGBitmapInfo(rawValue: (CGBitmapInfo.ByteOrder32Little.rawValue | CGImageAlphaInfo.PremultipliedFirst.rawValue) as UInt32)
        let newContext: CGContextRef! = CGBitmapContextCreate(baseAddress, width, height, bitsPerCompornent, bytesPerRow, colorSpace, bitmapInfo.rawValue) as CGContextRef!
        
        // Creates and returns a Quartz image from the pixel data in a bitmap graphics context.
        let imageRef: CGImageRef! = CGBitmapContextCreateImage(newContext!)
        
        // Unlock the Base Address
        CVPixelBufferUnlockBaseAddress(imageBuffer, 0)
        
        // Output to UIImage
        let resultImage: UIImage = UIImage(CGImage: imageRef)
    
        return resultImage
    }
    
}