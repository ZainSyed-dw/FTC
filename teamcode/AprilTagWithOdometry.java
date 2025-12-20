package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag w/ Odometry", group = "ZZZ")
public class AprilTagWithOdometry extends LinearOpMode {
    // Drive motors (using lf, rf, rb, lb configuration)
    private DcMotorEx lf, rf, rb, lb;
    
    // Odometry - using PinPoint computer and two dead wheels
    private DcMotorEx leftEncoder, rightEncoder;  // Two dead wheel encoders
    
    // AprilTag variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    // Constants
    private static final int TARGET_TAG_ID = 20;
    private static final double TILE_SIZE = 24.0; // 24 inches per FTC tile
    private static final double ROTATION_POWER = 0.3;
    
    // Odometry variables
    private double x, y, heading;
    private int lastLeftPos, lastRightPos;
    
    // GoBilda PinPoint with 35mm (1.38") dead wheels and 8192 CPR encoders
    private static final double WHEEL_DIAMETER = 1.38; // inches (35mm)
    private static final double TICKS_PER_REV = 8192.0; // For GoBilda PinPoint encoders
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
    
    // Robot dimensions - measure and update these
    private static final double TRACK_WIDTH = 12.0; // Distance between your two dead wheels (inches)

    @Override
    public void runOpMode() {
        initializeHardware();
        initAprilTag();
        
        telemetry.addData("Status", "Initialized. Press Play to start");
        telemetry.update();
        
        waitForStart();
        
        // Wait for camera to initialize
        while (opModeIsActive() && !visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }
        
        // Main loop
        while (opModeIsActive()) {
            updateOdometry();
            
            if (scanForAprilTag()) {
                telemetry.addData("Status", "AprilTag found! Rotating...");
                telemetry.update();
                
                // Wait 1.5 seconds before starting rotation
                sleep(1500);
                
                // Rotate until stopped
                rotateRobot(ROTATION_POWER);
                while (opModeIsActive()) {
                    updateOdometry();
                    telemetry.addData("Status", "Rotating...");
                    telemetry.addData("X", "%.1f in", x);
                    telemetry.addData("Y", "%.1f in", y);
                    telemetry.addData("Heading", "%.1f°", Math.toDegrees(heading));
                    telemetry.update();
                    idle();
                }
            }
            
            telemetry.addData("Status", "Searching for AprilTag ID: " + TARGET_TAG_ID);
            telemetry.addData("X", "%.1f in", x);
            telemetry.addData("Y", "%.1f in", y);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(heading));
            telemetry.update();
            
            sleep(100);
        }
        
        // Clean up
        stopMotors();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    private void initializeHardware() {
        // Initialize drive motors
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        
        // Initialize dead wheel encoders
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        
        // Reset encoders
        resetEncoders();
        
        // Set motor directions (adjust if needed)
        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        
        // Reset encoders
        resetEncoders();
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
    
    private void updateOdometry() {
        // Get current encoder positions
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        
        // Calculate delta movements in ticks
        int deltaLeft = leftPos - lastLeftPos;
        int deltaRight = rightPos - lastRightPos;
        
        // Update previous positions
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        
        // Convert ticks to inches
        double leftInches = deltaLeft / TICKS_PER_INCH;
        double rightInches = deltaRight / TICKS_PER_INCH;
        
        // Calculate change in heading (in radians)
        double deltaHeading = (rightInches - leftInches) / TRACK_WIDTH;
        
        // Update heading (normalize to -π to π)
        heading = (heading + deltaHeading) % (2 * Math.PI);
        if (heading > Math.PI) heading -= 2 * Math.PI;
        if (heading < -Math.PI) heading += 2 * Math.PI;
        
        // Calculate change in position (average of both wheels)
        double deltaDistance = (leftInches + rightInches) / 2.0;
        
        // Update global position (rotate by current heading)
        x += deltaDistance * Math.cos(heading);
        y += deltaDistance * Math.sin(heading);
    }
    
    private void resetEncoders() {
        if (leftEncoder != null && rightEncoder != null) {
            lastLeftPos = leftEncoder.getCurrentPosition();
            lastRightPos = rightEncoder.getCurrentPosition();
        }
        x = 0;
        y = 0;
        heading = 0;
    }
    
    private boolean scanForAprilTag() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }
        
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == TARGET_TAG_ID) {
                double distanceInches = detection.ftcPose.range;
                double distanceTiles = distanceInches / TILE_SIZE;
                
                telemetry.addLine("TAG DETECTED!");
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Distance", "%.2f tiles (%.1f in)", distanceTiles, distanceInches);
                telemetry.addData("X (in)", "%.1f", detection.ftcPose.x);
                telemetry.addData("Y (in)", "%.1f", detection.ftcPose.y);
                telemetry.addData("Bearing", "%.1f°", detection.ftcPose.bearing);
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
