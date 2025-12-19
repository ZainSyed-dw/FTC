package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "April Tag Test", group = "ZZZ")
public class AprilTagTest extends LinearOpMode {
    // Motor declarations (using your lf, rf, rb, lb configuration)
    private DcMotorEx lf, rf, rb, lb;  // lf=leftFront, rf=rightFront, rb=rightBack, lb=leftBack
    
    // AprilTag variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    // Constants
    private static final int TARGET_TAG_ID = 20;
    private static final double TILE_SIZE = 24.0; // 24 inches per FTC tile
    private static final double ROTATION_POWER = 0.3;
    private static final int SCAN_DURATION_MS = 3000; // 3 seconds of rotation
    private static final int SCAN_TIMEOUT_MS = 5000;  // 5 seconds to find tag
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        // Initialize AprilTag detection
        initAprilTag();
        
        // Wait for the driver to press PLAY
        telemetry.addData("Status", "Initialized. Press Play to start");
        telemetry.update();
        
        waitForStart();
        
        // Wait for camera to initialize
        while (!isStopRequested() && !visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }
        
        // Main autonomous routine
        if (opModeIsActive()) {
            // Continuously search for AprilTag until found
            boolean tagFound = false;
            
            while (opModeIsActive() && !tagFound) {
                tagFound = scanForAprilTag();
                
                if (!tagFound) {
                    telemetry.addData("Status", "Searching for AprilTag ID: " + TARGET_TAG_ID);
                    telemetry.update();
                    sleep(100); // Small delay to prevent busy-waiting
                }
            }
            
            // If we exited the loop because we found the tag
            telemetry.addData("Status", "AprilTag found! Preparing to spin...");
            telemetry.update();
            
            // Wait for 1-2 seconds before starting to spin
            sleep(1500);
            
            // Keep spinning until the program is stopped
            telemetry.addData("Status", "Spinning until stopped");
            telemetry.update();
            
            rotateRobot(ROTATION_POWER);
            
            // Keep spinning until the program is stopped
            while (opModeIsActive()) {
                idle();
            }
        }
        
        // Clean up
        if (visionPortal != null) {
            visionPortal.close();
        }
        stopMotors();
    }
    
    private void initializeHardware() {
        // Initialize drive motors with DcMotorEx using your configuration
        lf = hardwareMap.get(DcMotorEx.class, "lf");      // Left Front
        rf = hardwareMap.get(DcMotorEx.class, "rf");      // Right Front
        rb = hardwareMap.get(DcMotorEx.class, "rb");      // Right Back
        lb = hardwareMap.get(DcMotorEx.class, "lb");      // Left Back
        
        // Set motor directions (adjust if needed)
        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        
        // Set motor modes and behaviors
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void initAprilTag() {
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .build();
            
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                    .setAutoStopLiveView(true)
                    .build();
                    
            telemetry.addData("AprilTag", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize AprilTag: %s", e.getMessage());
        }
    }
    
    private void setMotorModes(DcMotor.RunMode mode) {
        lf.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
        lb.setMode(mode);
    }
    
    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        lf.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
    }
    
    private boolean scanForAprilTag() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }
        
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        // Process detections
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == TARGET_TAG_ID) {
                // Calculate distance in inches and convert to tiles
                double distanceInches = detection.ftcPose.range;
                double distanceTiles = distanceInches / TILE_SIZE;
                
                // Display detection info
                telemetry.addLine("TAG DETECTED!");
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Distance", "%.2f tiles (%.1f in)", distanceTiles, distanceInches);
                telemetry.addData("X (in)", "%.1f", detection.ftcPose.x);
                telemetry.addData("Y (in)", "%.1f", detection.ftcPose.y);
                telemetry.addData("Bearing", "%.1f°", detection.ftcPose.bearing);
                telemetry.update();
                
                return true;
            }
        }
        
        return false;
    }
    
    private void rotateRobot(double power) {
        // For mecanum wheels, all wheels rotate in the same direction to turn
        lf.setPower(power);    // Left front
        rf.setPower(-power);   // Right front
        rb.setPower(-power);   // Right back
        lb.setPower(power);    // Left back
    }
    
    private void stopMotors() {
        lf.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
}
