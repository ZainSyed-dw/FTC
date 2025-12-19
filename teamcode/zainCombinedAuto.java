package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Combined Auto", group = "Competition")
public class zainCombinedAuto extends LinearOpMode {
    // Hardware
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheel, intake;
    
    // AprilTag variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    
    // Constants
    private static final double DRIVE_SPEED = 0.5;
    private static final double INTAKE_POWER = 1.0;
    private static final double FLYWHEEL_POWER = 1.0;
    private static final int PAUSE_BETWEEN_ACTIONS_MS = 2000; // 2 seconds
    private static final int DESIRED_TAG_ID = 1; // Change to your target AprilTag ID
    
    @Override
    public void runOpMode() {
        // Initialize hardware and vision
        initializeHardware();
        initAprilTag();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        if (opModeIsActive()) {
            // Run the movement sequence
            runMovementSequence();
            
            // After movement sequence, use AprilTag for final positioning
            runAprilTagPositioning();
        }
        
        // Clean up
        visionPortal.close();
    }
    
    private void runMovementSequence() {
        // Move forward 10 inches with intake on
        intake.setPower(INTAKE_POWER);
        moveInches(10, 0, 0);
        sleep(PAUSE_BETWEEN_ACTIONS_MS);
        
        // Center itself (assuming center is at 0,0)
        moveToPosition(0, 0);
        sleep(PAUSE_BETWEEN_ACTIONS_MS);
        
        // Move backward 10 inches with flywheel on and intake off
        intake.setPower(0);
        flywheel.setPower(FLYWHEEL_POWER);
        moveInches(-10, 0, 0);
        sleep(PAUSE_BETWEEN_ACTIONS_MS);
        
        // Center itself
        moveToPosition(0, 0);
        sleep(PAUSE_BETWEEN_ACTIONS_MS);
        
        // Strafe left 10 inches with both intake and flywheel on
        intake.setPower(INTAKE_POWER);
        flywheel.setPower(FLYWHEEL_POWER);
        moveInches(0, 10, 0);
        sleep(PAUSE_BETWEEN_ACTIONS_MS);
        
        // Center itself
        moveToPosition(0, 0);
        sleep(PAUSE_BETWEEN_ACTIONS_MS);
        
        // Turn everything off and strafe right
        intake.setPower(0);
        flywheel.setPower(0);
        moveInches(0, -10, 0);
    }
    
    private void runAprilTagPositioning() {
        boolean targetFound = false;
        runtime.reset();
        
        // Look for the AprilTag for up to 5 seconds
        while (opModeIsActive() && !targetFound && runtime.seconds() < 5) {
            targetFound = lookForAprilTag();
            
            if (!targetFound) {
                // If no tag found, rotate slowly to find one
                rotateRobot(0.3);
                sleep(100);
            }
        }
        
        if (targetFound) {
            // Move toward the AprilTag
            moveToAprilTag();
        } else {
            telemetry.addData("AprilTag", "No target found within time limit");
            telemetry.update();
        }
        
        // Stop all motors
        stopMotors();
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
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(false)
          //      .setDrawAxis(true)
                .build();
        
        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag);
        
        visionPortal = builder.build();
    }
    
    private boolean lookForAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == DESIRED_TAG_ID) {
                telemetry.addData("Found AprilTag ID", detection.id);
                telemetry.addData("Range", "%5.1f inches", detection.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing);
                telemetry.update();
                return true;
            }
        }
        
        telemetry.addData("AprilTag", "No target found");
        telemetry.update();
        return false;
    }
    
    private void moveToAprilTag() {
        runtime.reset();
        
        // Try to move to the AprilTag for up to 5 seconds
        while (opModeIsActive() && runtime.seconds() < 5) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            boolean tagFound = false;
            
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == DESIRED_TAG_ID) {
                    double range = detection.ftcPose.range;
                    double bearing = detection.ftcPose.bearing;
                    
                    // Simple proportional control
                    double forwardPower = range * 0.05;
                    double turnPower = bearing * 0.01;
                    
                    // Move toward the AprilTag
                    driveRobot(forwardPower, 0, turnPower);
                    
                    // If we're close enough, stop
                    if (range < 6.0) { // 6 inches
                        stopMotors();
                        return;
                    }
                    
                    tagFound = true;
                    break;
                }
            }
            
            if (!tagFound) {
                break;
            }
            
            sleep(20); // Small delay to prevent busy-waiting
        }
        
        stopMotors();
    }
    
    private void moveInches(double x, double y, double rotation) {
        // Calculate motor powers based on movement
        double leftFrontPower = (y + x + rotation) * DRIVE_SPEED;
        double rightFrontPower = (y - x - rotation) * DRIVE_SPEED;
        double leftBackPower = (y - x + rotation) * DRIVE_SPEED;
        double rightBackPower = (y + x - rotation) * DRIVE_SPEED;
        
        // Set motor powers
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        
        // Calculate time needed to move the specified distance
        double distanceInches = Math.sqrt(x*x + y*y);
        long moveTimeMs = (long)((distanceInches / (DRIVE_SPEED * 12)) * 1000);
        
        // Wait for the movement to complete
        sleep(moveTimeMs);
        
        // Stop all motors
        stopMotors();
    }
    
    private void moveToPosition(double targetX, double targetY) {
        // Simple implementation - in a real scenario, you'd use odometry
        moveInches(-targetX, -targetY, 0);
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
