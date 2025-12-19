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

@Autonomous(name = "ZAIN AUTO TEST", group = "ZZZ")
public class zainBlueAuto extends LinearOpMode {
    // Hardware
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheel;

    private DcMotor intake  ;

    // AprilTag variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private ElapsedTime runtime = new ElapsedTime();

    // Constants
    private static final double DRIVE_SPEED = 0.5;
    private static final double INTAKE_POWER = 1.0;
    private static final double FLYWHEEL_POWER = 1.0;
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final double TILE_LENGTH = 24.0; // inches
    private static final double COUNTS_PER_INCH = 2.63; // For 86mm Swyft v2 wheels with 20:1 HD Hex Motors
    @Override
    public void runOpMode() {
        initializeHardware();
        initAprilTag();

        waitForStart();

        if (opModeIsActive()) {
            // 1. Move 4 tiles forward
            moveInches(0, 4 * TILE_LENGTH, 0);
            sleep(500);

            // 2. Move 1 tile left
            moveInches(-TILE_LENGTH, 0, 0);
            sleep(500);

            // 3. Rotate 45 degrees left (negative for counter-clockwise)
            rotateDegrees(-45);
            sleep(500);

            // 4. Scan for AprilTag
            boolean targetFound = false;
            runtime.reset();
            while (opModeIsActive() && !targetFound && runtime.seconds() < 5.0) {
                targetFound = lookForAprilTag();
                if (targetFound) {
                    // 5. Center on the field
                    centerOnField();
                    
                    // 6. Run the movement test sequence after centering
                    runMovementTest();
                    
                    // 7. Move to AprilTag (original behavior)
                    moveToAprilTag();
                }
                sleep(100);
            }
            
            stopMotors();
        }

        visionPortal.close();
    }
    
    /**
     * Runs a series of movement tests to verify robot functionality
     */
    private void runMovementTest() {
        telemetry.addData("Status", "Starting Movement Test");
        telemetry.update();
        
        // 1. Move 10 inches forward with intake on
        telemetry.addData("Test", "Moving forward 10 inches with intake on");
        telemetry.update();
        intake.setPower(INTAKE_POWER);
        moveInches(0, 10, 0);
        sleep(500);

        // 2. Reverse 10 inches (centering)
        telemetry.addData("Test", "Reversing 10 inches");
        telemetry.update();
        moveInches(0, -10, 0);
        sleep(500);

        // 3. Pause 2 seconds (intake off during this time)
        telemetry.addData("Test", "Pausing 2 seconds (intake off)");
        telemetry.update();
        intake.setPower(0);
        sleep(2000);

        // 4. Move backward 10 inches with flywheel at full power
        telemetry.addData("Test", "Moving backward 10 inches with flywheel on");
        telemetry.update();
        flywheel.setPower(FLYWHEEL_POWER);
        moveInches(0, -10, 0);
        sleep(500);

        // 5. Move forward 10 inches (centering)
        telemetry.addData("Test", "Moving forward 10 inches");
        telemetry.update();
        moveInches(0, 10, 0);
        sleep(500);

        // 6. Pause 2 seconds (turn off flywheel and intake)
        telemetry.addData("Test", "Pausing 2 seconds (all off)");
        telemetry.update();
        flywheel.setPower(0);
        intake.setPower(0);
        sleep(2000);

        // 7. Strafe left with both intake and flywheel on for 10 inches
        telemetry.addData("Test", "Strafing left 10 inches with intake and flywheel");
        telemetry.update();
        intake.setPower(INTAKE_POWER);
        flywheel.setPower(FLYWHEEL_POWER);
        moveInches(-10, 0, 0);
        sleep(500);

        // 8. Strafe right 10 inches (centering)
        telemetry.addData("Test", "Strafing right 10 inches");
        telemetry.update();
        moveInches(10, 0, 0);
        sleep(500);

        // 9. Pause 2 seconds (turn off flywheel and intake)
        telemetry.addData("Test", "Pausing 2 seconds (all off)");
        telemetry.update();
        flywheel.setPower(0);
        intake.setPower(0);
        sleep(2000);

        // 10. Strafe right (nothing on) 10 inches
        telemetry.addData("Test", "Strafing right 10 inches (no accessories)");
        telemetry.update();
        moveInches(10, 0, 0);
        sleep(500);

        // 11. Strafe left 10 inches (centering)
        telemetry.addData("Test", "Strafing left 10 inches");
        telemetry.update();
        moveInches(-10, 0, 0);
        sleep(500);

        // 12. Pause 2 seconds
        telemetry.addData("Test", "Pausing 2 seconds");
        telemetry.update();
        sleep(2000);

        // 13. Rotate left 360 degrees
        telemetry.addData("Test", "Rotating left 360 degrees");
        telemetry.update();
        rotateDegrees(360);
        sleep(500);

        // 14. Pause 2 seconds
        telemetry.addData("Test", "Pausing 2 seconds");
        telemetry.update();
        sleep(2000);

        // 15. Rotate right 360 degrees
        telemetry.addData("Test", "Rotating right 360 degrees");
        telemetry.update();
        rotateDegrees(-360);
        sleep(500);

        // 16. Test complete
        telemetry.addData("Status", "Movement Test Complete!");
        telemetry.update();
        sleep(2000);
    }

    private void centerOnField() {
        // Assuming we're at (-24, 96) after initial moves
        // Move to center of field (0, 48)
        double dx = 24.0; // inches to right
        double dy = -48.0; // inches back

        // Move to center
        moveInches(dx, dy, 0);
        sleep(500);

        // Face forward (0 degrees)
        rotateDegrees(45); // Since we were at -45 degrees
    }
    private void rotateRobot(double power) {
        // Set power to rotate the robot in place
        // Positive power rotates clockwise, negative rotates counter-clockwise
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }
    private boolean lookForAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == BLUE_GOAL_TAG_ID) {
                telemetry.addData("Found Blue Goal Tag ID", detection.id);
                telemetry.addData("Range", "%.1f inches", detection.ftcPose.range);
                telemetry.addData("Bearing", "%.1f°", detection.ftcPose.bearing);
                telemetry.update();
                return true;
            }
        }

        telemetry.addLine("Looking for AprilTag...");
        telemetry.update();
        return false;
    }

    private void moveToAprilTag() {
        boolean targetFound = false;
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 10.0) { // 10 second timeout
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == BLUE_GOAL_TAG_ID) {
                    double range = detection.ftcPose.range;
                    double bearing = detection.ftcPose.bearing;

                    // Simple proportional control
                    double forwardPower = range * 0.03; // Slower approach
                    double turnPower = bearing * 0.01;

                    // Move toward the AprilTag
                    driveRobot(forwardPower, 0, turnPower);

                    // If we're close enough, stop
                    if (range < 6.0) { // 6 inches
                        stopMotors();
                        return;
                    }

                    targetFound = true;
                    break;
                }
            }

            if (!targetFound) {
                // If we lost the tag, rotate to find it
                rotateRobot(0.2);
            }

            sleep(20);
        }

        stopMotors();
    }

    private void moveInches(double x, double y, double rotation) {
        // Convert inches to encoder counts
        int leftFrontTarget = (int)(y - x - rotation) * (int)COUNTS_PER_INCH;
        int rightFrontTarget = (int)(y + x + rotation) * (int)COUNTS_PER_INCH;
        int leftBackTarget = (int)(y + x - rotation) * (int)COUNTS_PER_INCH;
        int rightBackTarget = (int)(y - x + rotation) * (int)COUNTS_PER_INCH;

        // Set target positions
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + leftFrontTarget);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rightFrontTarget);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + leftBackTarget);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rightBackTarget);

        // Set to RUN_TO_POSITION mode
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving
        driveRobot(DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED);

        // Wait until all motors reach their target positions
        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() ||
                        leftBack.isBusy() || rightBack.isBusy())) {
            // Wait for movement to complete
        }

        // Stop all motion
        stopMotors();
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void rotateDegrees(double degrees) {
        // Convert degrees to encoder counts (adjust this based on your robot's configuration)
        // This is a simplified version - you'll need to tune these values
        double rotations = degrees / 90.0; // Approximate
        int counts = (int)(rotations * 1120); // 1120 encoder counts per motor revolution

        // Set target positions
        leftFront.setTargetPosition(leftFront.getCurrentPosition() - counts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + counts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() - counts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + counts);

        // Set to RUN_TO_POSITION mode
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving
        double speed = 0.3; // Slower for more precise rotation
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);

        // Wait until all motors reach their target positions
        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() ||
                        leftBack.isBusy() || rightBack.isBusy())) {
            // Wait for rotation to complete
        }

        // Stop all motion
        stopMotors();
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeHardware() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void driveRobot(double leftFrontPower, double rightFrontPower,
                            double leftBackPower, double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    private void driveRobot(double x, double y, double rotation) {
        double leftFrontPower = y - x - rotation;
        double rightFrontPower = y + x + rotation;
        double leftBackPower = y + x - rotation;
        double rightBackPower = y - x + rotation;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        driveRobot(leftFrontPower * DRIVE_SPEED,
                rightFrontPower * DRIVE_SPEED,
                leftBackPower * DRIVE_SPEED,
                rightBackPower * DRIVE_SPEED);
    }

    private void stopMotors() {
        driveRobot(0, 0, 0, 0);
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