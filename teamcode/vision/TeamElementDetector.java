package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class TeamElementDetector extends OpenCvPipeline {
    private OpenCvWebcam webcam;
    private final Object imgLock = new Object();
    private Mat lastFrame;
    
    // Camera settings
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;
    
    // Detection settings
    private Scalar lowerRed = new Scalar(0, 100, 100);
    private Scalar upperRed = new Scalar(10, 255, 255);
    private Scalar lowerBlue = new Scalar(100, 100, 100);
    private Scalar upperBlue = new Scalar(140, 255, 255);
    
    // Detection results
    private boolean isRedDetected = false;
    private boolean isBlueDetected = false;
    
    public TeamElementDetector(OpenCvWebcam webcam) {
        this.webcam = webcam;
        this.webcam.setPipeline(this);
    }
    
    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV color space for better color detection
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        
        // Create masks for red and blue
        Mat redMask = new Mat();
        Mat blueMask = new Mat();
        
        Core.inRange(hsv, lowerRed, upperRed, redMask);
        Core.inRange(hsv, lowerBlue, upperBlue, blueMask);
        
        // Check if we found any red or blue pixels
        isRedDetected = Core.countNonZero(redMask) > 1000; // Threshold can be adjusted
        isBlueDetected = Core.countNonZero(blueMask) > 1000;
        
        // Release resources
        hsv.release();
        redMask.release();
        blueMask.release();
        
        // Store the last processed frame (for debugging)
        synchronized (imgLock) {
            if (lastFrame != null) {
                lastFrame.release();
            }
            lastFrame = input.clone();
        }
        
        return input;
    }
    
    public boolean isRedDetected() {
        return isRedDetected;
    }
    
    public boolean isBlueDetected() {
        return isBlueDetected;
    }
    
    public void stop() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
        synchronized (imgLock) {
            if (lastFrame != null) {
                lastFrame.release();
                lastFrame = null;
            }
        }
    }
}
