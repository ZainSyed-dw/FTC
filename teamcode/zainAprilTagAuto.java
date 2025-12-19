package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Auto", group = "Competition")
public class zainAprilTagAuto extends LinearOpMode {
    // Hardware
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheel, intake;
    
    // AprilTag variables
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    // Constants
    private static final double DRIVE_SPEED = 0.5;
    private static final int DESIRED_TAG_ID = 1; // Change this to the AprilTag ID you want to target
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        // Initialize AprilTag detection
        initAprilTag();
        
        // Wait for the driver to press PLAY
        waitForStart();
        
        if (opModeIsActive()) {
            // Run the autonomous routine
            runAutonomous();
        }
        
        // Clean up
        visionPortal.close();
    }
    
    private void initializeHardware() {
        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Initialize other motors
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        
        // Set motor directions (adjust if needed)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        
        // Set motor modes
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void initAprilTag() {
        // Create the AprilTag processor with minimal configuration
        aprilTag = new AprilTagProcessor.Builder().build();
        
        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag);
        
        visionPortal = builder.build();
    }
    
    private void runAutonomous() {
        boolean targetFound = false;
        
        // Look for the AprilTag
        while (opModeIsActive() && !targetFound) {
            targetFound = lookForAprilTag();
            
            if (targetFound) {
                // Move toward the AprilTag
                moveToAprilTag();
            } else {
                // If no tag found, rotate slowly to find one
                rotateRobot(0.3);
                sleep(100);
            }
            
            telemetry.update();
        }
        
        // Stop all motors
        stopMotors();
    }
    
    private boolean lookForAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == DESIRED_TAG_ID) {
                telemetry.addData("Found AprilTag ID", detection.id);
                telemetry.addData("Range", "%5.1f inches", detection.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing);
                return true;
            }
        }
        
        telemetry.addData("AprilTag", "No target found");
        return false;
    }
    
    private void moveToAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == DESIRED_TAG_ID) {
                double range = detection.ftcPose.range;
                double bearing = detection.ftcPose.bearing;
                
                // Simple proportional control
                double forwardPower = range * 0.05; // Adjust the multiplier as needed
                double turnPower = bearing * 0.01; // Adjust the multiplier as needed
                
                // Move toward the AprilTag
                driveRobot(forwardPower, 0, turnPower);
                
                // If we're close enough, stop
                if (range < 6.0) { // 6 inches
                    stopMotors();
                    // Perform any end-of-mission actions here
                    return;
                }
                
                break;
            }
        }
    }
    
    private void driveRobot(double x, double y, double rotation) {
        double leftFrontPower = (y + x + rotation) * DRIVE_SPEED;
        double rightFrontPower = (y - x - rotation) * DRIVE_SPEED;
        double leftBackPower = (y - x + rotation) * DRIVE_SPEED;
        double rightBackPower = (y + x - rotation) * DRIVE_SPEED;
        
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
    
    private void rotateRobot(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }
    
    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    
    private void setMotorModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }
    
    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }
}
