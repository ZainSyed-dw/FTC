package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Run Movement Test")
public class runMovementTest extends LinearOpMode {
    // Constants
    private static final double INTAKE_POWER = 1.0;
    private static final double FLYWHEEL_POWER = 0.7;
    private static final double DRIVE_SPEED = 0.6;
    private static final double TICKS_PER_REV = 28.0; // HD Hex 20:1 motor
    private static final double WHEEL_DIAMETER_INCHES = 3.38583; // Adjust based on your wheel
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    
    // Motors
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor intake;
    private DcMotorEx flywheel;  // Using DcMotorEx for velocity control

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        // 1. Move 10 inches forward with intake on
        intake.setPower(INTAKE_POWER);
        moveInches(0, 10, 0);

        // 2. Reverse 10 inches (centering)
        moveInches(0, -10, 0);

        // 3. Pause 2 seconds (intake off during this time)
        intake.setPower(0);
        sleep(2000);

        // 4. Move backward 10 inches with flywheel at 1450 RPM
        double ticksPerSecond = (1450 * TICKS_PER_REV) / 60.0;
        flywheel.setVelocity(ticksPerSecond);  // Now using DcMotorEx's setVelocity
        moveInches(0, -10, 0);

        // 5. Move forward 10 inches (centering)
        moveInches(0, 10, 0);

        // 6. Pause 2 seconds (turn off flywheel and intake)
        flywheel.setPower(0);
        intake.setPower(0);
        sleep(2000);

        // 7. Strafe left with both intake and flywheel on for 10 inches
        intake.setPower(INTAKE_POWER);
        flywheel.setPower(FLYWHEEL_POWER);
        moveInches(-10, 0, 0);

        // 8. Strafe right 10 inches (centering)
        moveInches(10, 0, 0);

        // 9. Pause 2 seconds (turn off flywheel and intake)
        flywheel.setPower(0);
        intake.setPower(0);
        sleep(2000);

        // 10. Strafe right (nothing on) 10 inches
        moveInches(10, 0, 0);

        // 11. Strafe left 10 inches (centering)
        moveInches(-10, 0, 0);

        // 12. Pause 2 seconds
        sleep(2000);

        // 13. Rotate left 360 degrees
        rotateDegrees(360);

        // 14. Pause 2 seconds
        sleep(2000);

        // 15. Rotate right 360 degrees
        rotateDegrees(-360);

        // 16. Stop all motors
        stopMotors();
    }
    
    private void initializeHardware() {
        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        
        // Initialize other motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Enable encoder mode for velocity control
        
        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        
        // Set motor modes
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void setMotorModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
    }
    
    private void moveInches(double x, double y, double rotation) {
        // Calculate motor powers for mecanum drive
        double leftFrontPower = y + x + rotation;
        double leftRearPower = y - x + rotation;
        double rightFrontPower = y - x - rotation;
        double rightRearPower = y + x - rotation;
        
        // Normalize powers
        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower)),
                                Math.max(Math.abs(rightFrontPower), Math.abs(rightRearPower)));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftRearPower /= maxPower;
            rightFrontPower /= maxPower;
            rightRearPower /= maxPower;
        }
        
        // Set target positions
        int targetLeftFront = leftFront.getCurrentPosition() + (int)((y + x + rotation) * TICKS_PER_INCH);
        int targetLeftRear = leftRear.getCurrentPosition() + (int)((y - x + rotation) * TICKS_PER_INCH);
        int targetRightFront = rightFront.getCurrentPosition() + (int)((y - x - rotation) * TICKS_PER_INCH);
        int targetRightRear = rightRear.getCurrentPosition() + (int)((y + x - rotation) * TICKS_PER_INCH);
        
        // Set target positions
        leftFront.setTargetPosition(targetLeftFront);
        leftRear.setTargetPosition(targetLeftRear);
        rightFront.setTargetPosition(targetRightFront);
        rightRear.setTargetPosition(targetRightRear);
        
        // Set motor power
        leftFront.setPower(DRIVE_SPEED * leftFrontPower);
        leftRear.setPower(DRIVE_SPEED * leftRearPower);
        rightFront.setPower(DRIVE_SPEED * rightFrontPower);
        rightRear.setPower(DRIVE_SPEED * rightRearPower);
        
        // Wait for motors to reach target
        while (opModeIsActive() && 
               (leftFront.isBusy() || leftRear.isBusy() || 
                rightFront.isBusy() || rightRear.isBusy())) {
            // Wait for movement to complete
        }
        
        // Stop motors
        setAllDrivePower(0);
    }
    
    private void rotateDegrees(double degrees) {
        // Convert degrees to inches (adjust this based on your robot's dimensions)
        double wheelBaseWidth = 16.0; // Adjust this to your robot's wheelbase width
        double arcLength = (Math.PI * wheelBaseWidth * Math.abs(degrees)) / 360.0;
        int target = (int)(arcLength * TICKS_PER_INCH);
        
        // Set target positions
        if (degrees > 0) {
            // Rotate left (counter-clockwise)
            setAllDrivePower(DRIVE_SPEED);
            leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
            leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
            rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
            rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
        } else {
            // Rotate right (clockwise)
            setAllDrivePower(-DRIVE_SPEED);
            leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
            leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
            rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
            rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
        }
        
        // Set motor modes
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Wait for motors to reach target
        while (opModeIsActive() && 
               (leftFront.isBusy() || leftRear.isBusy() || 
                rightFront.isBusy() || rightRear.isBusy())) {
            // Wait for rotation to complete
        }
        
        // Stop motors
        setAllDrivePower(0);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void setAllDrivePower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }
    
    private void stopMotors() {
        // Stop all motors
        setAllDrivePower(0);
        intake.setPower(0);
        flywheel.setPower(0);
    }
}
