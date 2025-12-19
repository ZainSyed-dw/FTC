package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Zain Auto", group = "Competition")
public class zainAuto extends LinearOpMode {
    // Hardware components
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    
    // Constants
    public static double DRIVE_SPEED = 0.5; // Max wheel speed
    public static double INTAKE_POWER = 1.0;
    public static double FLYWHEEL_POWER = 1.0;
    public static int PAUSE_BETWEEN_ACTIONS_MS = 2000; // 2 seconds
    public static double INCHES_PER_TICK = 1.0; // Adjust this based on your robot's configuration
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        if (opModeIsActive()) {
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
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set zero power behavior
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        long moveTimeMs = (long)((distanceInches / (DRIVE_SPEED * 12)) * 1000); // Convert to seconds then to milliseconds
        
        // Wait for the movement to complete
        sleep(moveTimeMs);
        
        // Stop all motors
        stopMotors();
    }
    
    private void moveToPosition(double targetX, double targetY) {
        // Simple implementation - in a real scenario, you'd use odometry
        // For now, we'll just move to what we think is center (0,0)
        moveInches(-targetX, -targetY, 0);
    }
    
    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
