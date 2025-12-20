package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.ShootingTuning.*;

@Autonomous(name = "OFFICAL AUTO ZAIN", group = "ZZZ")
public class zainBlueAuto extends LinearOpMode {
    // Motors
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheel, intake;
    
    // Constants
    private static final double TILE_SIZE = 24.0; // inches
    private static final double WHEEL_DIAMETER_MM = 86.0; // mm for Swerve v2 wheels
    private static final double WHEEL_DIAMETER_IN = WHEEL_DIAMETER_MM / 25.4; // Convert to inches
    private static final double COUNTS_PER_MOTOR_REV = 28.0; // For Swerve v2 with 19.2:1 ratio
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_IN * Math.PI);
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.5;
    
    // Flywheel power calculation
    private static final double SHOOTING_POWER = TARGET_FLYWHEEL_RPM / MAX_RPM;
    
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Sequence 1: Move to shooting position and shoot 3 balls
            runSequence1();
            
            // Sequence 1.5: Rotate to face driver (315° to 270° = 45° right)
            runSequence1_5();
            
            // Sequence 2: Move to position C, pick up balls, and shoot
            runSequence2();

            runSequence1_5();
            
            //Sequence 3: Move to position C, pick up balls, and shoot
            runSequence3();
            
            runSequence1_5();

            runSequence4();
            
            // Sequence 5: Rotate to 270° and then to -270°
            runSequence5();
            
            telemetry.addData("Status", "All sequences complete!");
            telemetry.update();
        }
    }
    
    /**
     * Sequence 1: Move to shooting position and shoot 3 preloaded balls
     */
    private void runSequence1() {
        telemetry.addData("Status", "Starting Sequence 1: Move to shooting position");
        telemetry.update();
        
        // Move from F3 to B3 (4 tiles forward) - Robot centric
        moveInches(4 * TILE_SIZE, 0, DRIVE_SPEED);
        sleep(500);
        
        // Rotate to face the goal (230° right from start)
        telemetry.addData("Status", "Rotating 230° right to face goal");
        telemetry.update();
        rotateDegrees(230, TURN_SPEED);
        sleep(500);
        
        // Execute the shooting sequence for 3 balls
        telemetry.addData("Status", "Executing shooting sequence");
        telemetry.update();
        shootThreeBalls();
        
        telemetry.addData("Status", "Sequence 1 complete - All 3 balls shot");
        telemetry.update();
    }
    
    /**
     * Sequence 1.5: Rotate to face driver (230° to 270° = 40° right)
     * Intake will be facing the driver after this rotation
     */
    private void runSequence1_5() {
        telemetry.addData("Status", "Starting Sequence 1.5: Rotating to face driver");
        telemetry.update();
        
        // Rotate 40° right (from 230° to 270°)
        telemetry.addData("Status", "Rotating from 230° to 270° (intake facing driver)");
        telemetry.update();
        rotateDegrees(40, TURN_SPEED);
        sleep(500);
        
        telemetry.addData("Status", "Sequence 1.5 complete - Intake now facing driver");
        telemetry.update();
    }
    
    /**
     * Sequence 2: Move to position C, pick up balls, and shoot
     */
    private void runSequence2() {
        telemetry.addData("Status", "Starting Sequence 2: Moving to position C");
        telemetry.update();
        
        // Strafe right to middle of C3 (1 tile = 24 inches) - Positive = right
        telemetry.addData("Status", "Moving to middle of C3");
        telemetry.update();
        strafeInches(1.5 * TILE_SIZE, 0.6);  // 1 tile = 24 inches to C3
        
        // Move forward to collect balls
        telemetry.addData("Status", "Moving forward to collect balls");
        telemetry.update();
        moveInches(20, 0, 0.5);  // Move forward 1 foot to collect balls
        
        // Run intake while moving forward
        telemetry.addData("Status", "Running intake to collect balls");
        telemetry.update();
        setIntakePower(INTAKE_POWER);
        
        // Move forward a bit more to ensure collection
        moveInches(10, 0, 0.3);  // Move forward another foot slowly
        
        
        
        // Move back to clear the balls
        telemetry.addData("Status", "Backing up from balls");
        telemetry.update();
        moveInches(-30, 0, 0.6);  // Move back 1 foot
       
        // Stop intake
        setIntakePower(0);
        
        // Strafe right to return to shooting line (B3)
        telemetry.addData("Status", "Returning to shooting line");
        telemetry.update();
        strafeInches(-1.5 * TILE_SIZE, 0.6);  // Strafe right 1.5 tiles
        
   
        
       // Rotate to face the goal (270° to 230° = 40° left)
telemetry.addData("Status", "Rotating to face goal");
telemetry.update();
rotateDegrees(-40, TURN_SPEED);  // Rotate 40° left to face 230°
        
        // Shoot the collected balls
        telemetry.addData("Status", "Shooting collected balls");
        telemetry.update();
        shootThreeBalls();
        
        telemetry.addData("Status", "Sequence 2 complete - All collected balls shot");
        telemetry.update();


    }
     /**
     * Sequence 3: Move to position C, pick up balls, and shoot
     */
    private void runSequence3() {
        telemetry.addData("Status", "Starting Sequence 3: Moving to position C");
        telemetry.update();
        
        // Strafe right to middle of C3 (1 tile = 24 inches) - Positive = right
        telemetry.addData("Status", "Moving to middle of C3");
        telemetry.update();
        strafeInches(2.5 * TILE_SIZE, 0.6);  // 1 tile = 24 inches to C3
        
        // Move forward to collect balls
        telemetry.addData("Status", "Moving forward to collect balls");
        telemetry.update();
        moveInches(20, 0, 0.5);  // Move forward 1 foot to collect balls
        
        // Run intake while moving forward
        telemetry.addData("Status", "Running intake to collect balls");
        telemetry.update();
        setIntakePower(INTAKE_POWER);
        
        // Move forward a bit more to ensure collection
        moveInches(10, 0, 0.3);  // Move forward another foot slowly
        
        
        
        // Move back to clear the balls
        telemetry.addData("Status", "Backing up from balls");
        telemetry.update();
        moveInches(-30, 0, 0.6);  // Move back 1 foot
       
        // Stop intake
        setIntakePower(0);
        
        // Strafe right to return to shooting line (B3)
        telemetry.addData("Status", "Returning to shooting line");
        telemetry.update();
        strafeInches(-2.5 * TILE_SIZE, 0.6);  // Strafe right 1.5 tiles
        
   
        
       // Rotate to face the goal (270° to 230° = 40° left)
telemetry.addData("Status", "Rotating to face goal");
telemetry.update();
rotateDegrees(-40, TURN_SPEED);  // Rotate 40° left to face 230°
        
        // Shoot the collected balls
        telemetry.addData("Status", "Shooting collected balls");
        telemetry.update();
        shootThreeBalls();
        
        telemetry.addData("Status", "Sequence 3 complete - All collected balls shot");
        telemetry.update();
    }
    /**
     * Sequence 4: Move to position C, pick up balls, and shoot
     */
    private void runSequence4() {
        telemetry.addData("Status", "Starting Sequence 4: Moving to position C");
        telemetry.update();
        
        // Strafe right to middle of C3 (1 tile = 24 inches) - Positive = right
        telemetry.addData("Status", "Moving to middle of C3");
        telemetry.update();
        strafeInches(3.5 * TILE_SIZE, 0.6);  // 1 tile = 24 inches to C3
        
        // Move forward to collect balls
        telemetry.addData("Status", "Moving forward to collect balls");
        telemetry.update();
        moveInches(20, 0, 0.5);  // Move forward 1 foot to collect balls
        
        // Run intake while moving forward
        telemetry.addData("Status", "Running intake to collect balls");
        telemetry.update();
        setIntakePower(INTAKE_POWER);
        
        // Move forward a bit more to ensure collection
        moveInches(10, 0, 0.3);  // Move forward another foot slowly
        
        
        
        // Move back to clear the balls
        telemetry.addData("Status", "Backing up from balls");
        telemetry.update();
        moveInches(-30, 0, 0.6);  // Move back 1 foot
       
        // Stop intake
        setIntakePower(0);
        
        // Strafe right to return to shooting line (B3)
        telemetry.addData("Status", "Returning to shooting line");
        telemetry.update();
        strafeInches(-3.5 * TILE_SIZE, 0.6);  // Strafe right 1.5 tiles
        
   
        
       // Rotate to face the goal (270° to 230° = 40° left)
telemetry.addData("Status", "Rotating to face goal");
telemetry.update();
rotateDegrees(-40, TURN_SPEED);  // Rotate 40° left to face 230°
        
        // Shoot the collected balls
        telemetry.addData("Status", "Shooting collected balls");
        telemetry.update();
        shootThreeBalls();
        
        telemetry.addData("Status", "Sequence 4 complete - All collected balls shot");
        telemetry.update();
    }
    private void initializeHardware() {
        // Initialize drive motors for Swerve v2
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Initialize other mechanisms
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        
        // Configure motor directions for Swerve v2
        // Note: Adjust these based on your specific motor configuration
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Configure flywheel for velocity control
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set motor modes
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set zero power behavior
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /**
     * Strafe the robot robot-centrically (convenience method that uses moveInches)
     * @param inches Positive = right, Negative = left (in inches)
     * @param speed Motor power (0.0 to 1.0)
     */
    private void strafeInches(double inches, double speed) {
        moveInches(0, inches, speed);
    }
    
    /**
     * Move the robot robot-centrically (forward/back, strafe left/right)
     * @param forward Positive = forward, Negative = backward (in inches)
     * @param strafe Positive = right, Negative = left (in inches)
     * @param speed Motor power (0.0 to 1.0)
     */
    private void moveInches(double forward, double strafe, double speed) {
        // Calculate motor powers for mecanum drive (robot-centric)
        double leftFrontPower = forward + strafe;
        double rightFrontPower = forward - strafe;
        double leftBackPower = forward - strafe;
        double rightBackPower = forward + strafe;
        
        // Normalize wheel speeds to maintain consistent speed
        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                           Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        
        // Apply speed
        leftFrontPower *= speed;
        rightFrontPower *= speed;
        leftBackPower *= speed;
        rightBackPower *= speed;
        
        // Set motor powers
        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        
        // Calculate time needed to move the specified distance
        double distance = Math.sqrt(forward * forward + strafe * strafe);
        long moveTimeMs = (long)((distance / (speed * 12)) * 1000); // 12 inches/second at full speed
        
        // Move for the calculated time
        sleep(moveTimeMs);
        
        // Stop all motors
        setMotorPowers(0, 0, 0, 0);
    }
    
    /**
     * Rotate the robot in place (robot-centric)
     * @param degrees Positive = clockwise, Negative = counter-clockwise (in degrees)
     * @param speed Motor power (0.0 to 1.0)
     */
    private void rotateDegrees(double degrees, double speed) {
        // Calculate motor powers for rotation
        double leftPower = -Math.signum(degrees) * speed;
        double rightPower = Math.signum(degrees) * speed;
        
        // Calculate time needed to rotate (empirically determined)
        // Adjust the 2000 value based on your robot's rotation speed
        long rotateTimeMs = (long)(Math.abs(degrees) * 2000 / 90.0);
        
        // Set motor powers for rotation
        setMotorPowers(leftPower, rightPower, leftPower, rightPower);
        
        // Rotate for the calculated time
        sleep(rotateTimeMs);
        
        // Stop all motors
        setMotorPowers(0, 0, 0, 0);
    }
    
    /**
     * Shoots three balls using the specified sequence:
     * 1. Run intake for initial time
     * 2. Spin up flywheel to target RPM
     * 3. For each ball:
     *    - First two balls: Short reverse pulse
     *    - Third ball: Longer reverse
     *    - Wait for ball to be detected as shot (RPM change)
     *    - If ball not detected, shake and retry
     */
    private void shootThreeBalls() {
        ElapsedTime sequenceTimer = new ElapsedTime();
        sequenceTimer.reset();
        
        try {
            // Step 1: Run intake for initial time
            telemetry.addData("Status", "Starting intake");
            setIntakePower(INTAKE_POWER);
            sleep((long)(INITIAL_INTAKE_TIME * 1000));
            
            // Step 2: Start flywheel and wait for target RPM
            telemetry.addData("Status", "Spinning up flywheel");
            setFlywheelRPM(TARGET_FLYWHEEL_RPM);
            waitForFlywheelRPM(TARGET_FLYWHEEL_RPM);
            
            // Shoot 3 balls with the specified sequence
            for (int ball = 0; ball < 3 && opModeIsActive(); ball++) {
                // For first two balls, do short reverse pulse
                if (ball < 2) {
                    // Reverse intake briefly
                    setIntakePower(REVERSE_INTAKE_POWER);
                    sleep((long)(INTAKE_REVERSE_SHORT * 1000));
                    
                    // Return to normal intake
                    setIntakePower(INTAKE_POWER);
                    
                    // Wait for RPM to stabilize
                    waitForFlywheelRPM(TARGET_FLYWHEEL_RPM);
                } else {
                    // For the third ball, do a longer reverse
                    setIntakePower(REVERSE_INTAKE_POWER);
                    sleep((long)(INTAKE_REVERSE_LONG * 1000));
                    setIntakePower(INTAKE_POWER);
                }
                
                // Wait for ball to be detected as shot (RPM change)
                if (!waitForBallShot()) {
                    telemetry.addData("Status", "Ball " + (ball + 1) + " not detected, shaking...");
                    telemetry.update();
                    shakeRobot();
                    telemetry.addData("Status", "Ball " + (ball + 1) + " not shot, moving to next ball");
                    telemetry.update();
                    break;
                }
                
                telemetry.addData("Status", "Ball " + (ball + 1) + " shot successfully");
                telemetry.update();
            }
            
        } finally {
            // Always ensure motors are stopped
            stopAllMotors();
        }
    }
    
    private void setIntakePower(double power) {
        intake.setPower(power);
    }
    
    private void stopAllMotors() {
        flywheel.setPower(0);
        intake.setPower(0);
        setMotorPowers(0, 0, 0, 0);
    }
    
    private void waitForFlywheelRPM(double targetRPM) {
        ElapsedTime timer = new ElapsedTime();
        double currentRPM;
        
        do {
            currentRPM = getFlywheelRPM();
            telemetry.addData("Status", "Waiting for RPM: %.1f/%.1f", currentRPM, targetRPM);
            telemetry.update();
            sleep(50);
            
            if (timer.seconds() > MAX_SHOOTING_TIME) {
                telemetry.addData("Error", "Timeout waiting for flywheel RPM");
                telemetry.update();
                break;
            }
        } while (opModeIsActive() && currentRPM < RPM_THRESHOLD);
    }
    
    private boolean waitForBallShot() {
        double initialRPM = getFlywheelRPM();
        ElapsedTime timer = new ElapsedTime();
        
        while (opModeIsActive() && timer.seconds() < 3.0) { // Max 3 seconds to detect shot
            double currentRPM = getFlywheelRPM();
            double rpmChange = Math.abs(currentRPM - initialRPM);
            
            if (rpmChange > RPM_CHANGE_THRESHOLD) {
                return true; // Ball was shot
            }
            
            telemetry.addData("Status", "Waiting for ball shot. RPM Change: %.1f", rpmChange);
            telemetry.update();
            sleep(50);
        }
        
        return false; // Ball not detected as shot
    }
    
    private void shakeRobot() {
        for (int i = 0; i < SHAKE_CYCLES && opModeIsActive(); i++) {
            // Move up
            setMotorPowers(SHAKE_POWER, SHAKE_POWER, SHAKE_POWER, SHAKE_POWER);
            sleep((long)(SHAKE_DURATION * 1000));
            
            // Move down
            setMotorPowers(-SHAKE_POWER, -SHAKE_POWER, -SHAKE_POWER, -SHAKE_POWER);
            sleep((long)(SHAKE_DURATION * 1000));
        }
        
        // Stop all drive motors
        setMotorPowers(0, 0, 0, 0);
    }
    
    /**
     * Sequence 5: Rotate to 270° and then to -270°
     */
    private void runSequence5() {
        telemetry.addData("Status", "Starting Sequence 5: Rotating to 270° and -270°");
        telemetry.update();
        
        // Rotate to 270° (90° left from current position)
        telemetry.addData("Status", "Rotating to 270°");
        telemetry.update();
        rotateDegrees(270, TURN_SPEED);
        sleep(1000);
       
        
        telemetry.addData("Status", "Sequence 5 complete - Rotation sequence finished");
        telemetry.update();
    }
    
    // Helper methods
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
    
    private void setTargetPositions(int lf, int rf, int lb, int rb) {
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + lf);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rf);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + lb);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rb);
    }
    
    private void setMotorPowers(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }
    
    private boolean areMotorsBusy() {
        return leftFront.isBusy() || rightFront.isBusy() || 
               leftBack.isBusy() || rightBack.isBusy();
    }
    
    private void setFlywheelRPM(double targetRPM) {
        // Convert RPM to encoder ticks per second
        double ticksPerSecond = (targetRPM * COUNTS_PER_MOTOR_REV) / 60.0;
        
        // Set the velocity
        flywheel.setVelocity(ticksPerSecond);
    }
    
    private double getFlywheelRPM() {
        // Get current velocity in ticks per second and convert to RPM
        double ticksPerSecond = flywheel.getVelocity();
        return (ticksPerSecond * 60.0) / COUNTS_PER_MOTOR_REV;
    }
}
