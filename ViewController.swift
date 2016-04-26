//
//  ViewController.swift
//  ObjectDetectionSwift
//
//  Created by Maitham Dib on 22/02/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//



import UIKit
import AVFoundation
import GLKit


class ViewController: UIViewController, AVCaptureVideoDataOutputSampleBufferDelegate{

    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        if initCamera() {
            // If we can initialise camera start the session
            mySession.startRunning()
        }
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    @IBOutlet weak var imageView: UIImageView!
    
    // create instances from AV foundation
    var mySession: AVCaptureSession!
    var myDevice: AVCaptureDevice!
    var myOutput: AVCaptureVideoDataOutput!
    var myFocus: AVCaptureFocusMode!
    
    //*************************************************************************************Compute Projection Matrix for Iphone 6 camera

//    func calculateProjectionMatrix(){
//        let projectionMatrix = cameraCalibration.createCameraProjection(myDevice, output:self.imageView , zNear: 5.0, zFar: 1000.0)
//        print("\(projectionMatrix)")
//    }
//    
    
    // Create instance of Image Processing Class
    let detectCars = ImageProcessing()
    
    
    // Function to initate camera and appropriate buffers to prevent lag etc.
    func initCamera() -> Bool {
        // Create instance of capture Sessions
        mySession = AVCaptureSession()
        
        // Set output of session so it is suitable for required output Photo = high quality photo output
        mySession.sessionPreset = AVCaptureSessionPreset1280x720

        // Returns array of Available devices that phone/system has
        let devices = AVCaptureDevice.devices()
        
        // Look through Array and Find the Back Camera and cast it as the AVCapture Device that we will use for object Detection, make sure to check if device array is empty
        for device in devices {
            if(device.position == AVCaptureDevicePosition.Back){
                myDevice = device as! AVCaptureDevice
            }
        }
        if myDevice == nil {
            return false
        }
        
        myFocus = AVCaptureFocusMode.AutoFocus
        
        // Check to see if we can capture input from the specific Device
        var myInput: AVCaptureDeviceInput! = nil
        do {
            myInput = try AVCaptureDeviceInput(device: myDevice) as AVCaptureDeviceInput
        } catch let error {
            print(error)
        }
        
        // Check to see if we can add input to the specific session so we can write out respective rectangles and circles to show object is recognised
        if mySession.canAddInput(myInput) {
            mySession.addInput(myInput)
        } else {
            return false
        }
        
        // Create instance of AVcapture Output that we can output to, specify the buffer pixel Format Type Key
        myOutput = AVCaptureVideoDataOutput()
        myOutput.videoSettings = [ kCVPixelBufferPixelFormatTypeKey: Int(kCVPixelFormatType_32BGRA) ]
        
        
        
        // acquire access to device configuration and set minimum and maximum allowed framerate
        do {
            try myDevice.lockForConfiguration()
            myDevice.activeVideoMinFrameDuration = CMTimeMake(1, 30);
            myDevice.activeVideoMaxFrameDuration = CMTimeMake(1, 30);
            myDevice.unlockForConfiguration()
        } catch let error {
            print("lock error: \(error)")
            return false
        }
        
        //If the queue is blocked when new frames are captured, those frames will be automatically dropped at a time determined by the value of the alwaysDiscardsLateVideoFrames property. This allows you to process existing frames on the same queue without having to manage the potential memory usage increases that would otherwise occur when that processing is unable to keep up with the rate of incoming frames.
        let queue: dispatch_queue_t = dispatch_queue_create("myqueue",  nil)
        myOutput.setSampleBufferDelegate(self,queue: queue)

        
        // See above comment, but this is to prevent frames from being stuck to the main Queue
        myOutput.alwaysDiscardsLateVideoFrames = true
        
        // Make sure a given output can be added to the session
        if mySession.canAddOutput(myOutput) {
            mySession.addOutput(myOutput)
        } else {
            return false
        }
        
        // detectCars
        for connection in myOutput.connections {
            if let conn = connection as? AVCaptureConnection {
                if conn.supportsVideoOrientation {
                    conn.videoOrientation = AVCaptureVideoOrientation.LandscapeLeft
                }
            }
        }

    
        return true
    }
    
    
  
    
    
    // Function to capture Output
    func captureOutput(captureOutput: AVCaptureOutput!, didOutputSampleBuffer sampleBuffer: CMSampleBuffer!, fromConnection connection: AVCaptureConnection!)
    {
        dispatch_sync(dispatch_get_main_queue(), {
            let image = CameraUtil.imageFromSampleBuffer(sampleBuffer)
            let image2 = CameraUtil.imageFromSampleBuffer(sampleBuffer)
            

            // Calculate optic Flow and Merge with Image 2.
            let mergedImage = self.detectCars.detectLanesGetOpticFlow(image);
//            let carImage = self.detectCars.recognizeCars(detectedVanishingPoint)

//            let addedLines = self.detectCars.carryOutHough(image)
            //output the CarImage to the ImageView to Show User
//            self.imageView.contentMode = .ScaleAspectFill
            self.imageView.image = mergedImage
            });
        
    }
    
    

}

