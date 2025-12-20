package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.ShootingTuning.*;

/**
 * zainBlueAuto - Full Road Runner autonomous with trajectory following
 * Uses Road Runner's advanced features: trajectory building, action sequences, 
 * motion profiling, and Pinpoint odometry integration
 */
@Autonomous(name = "OFFICAL AUTO ZAIN - RR", group = "ZZZ")
public class zainBlueAuto extends LinearOpMode {
    // Drive system
    private MecanumDrive drive;
    
    // Game elements
    private DcMotorEx flywheel, intake;
    
    // Constants
    private static final double TILE_SIZE = 24.0; // inches
    
    // Road Runner constraints
    private static final double MAX_VEL = 50.0; // in/s
    private static final double MAX_ACCEL = 40.0; // in/s^2
    private static final double MAX_ANG_VEL = Math.toRadians(180); // rad/s
    private static final double MAX_ANG_ACCEL = Math.toRadians(180); // rad/s^2
    private static final double TRACK_WIDTH = 15.0; // inches
    
    // Starting pose for blue alliance (F3 position)
    private static final Pose2d START_POSE = new Pose2d(-36, -60, Math.toRadians(90));
    
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize telemetry with dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        initializeHardware();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Pose", START_POSE);
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Execute full Road Runner autonomous
            executeRoadRunnerAuto();
            
            telemetry.addData("Status", "All sequences complete!");
            telemetry.update();
        }
    }
    
    private void initializeHardware() {
        // Initialize drive with Pinpoint localizer
        drive = new MecanumDrive(hardwareMap, START_POSE);
        
        // Initialize game elements
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        
        // Configure flywheel for velocity control
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        telemetry.log().add("Road Runner drive initialized");
        telemetry.log().add("Starting pose: " + START_POSE);
    }
    
    private void executeRoadRunnerAuto() {
        // Phase 1: Move to shooting position
        telemetry.addData("Status", "Phase 1: Moving to shooting position");
        telemetry.update();
        
        Actions.runBlocking(
            drive.actionBuilder(START_POSE)
                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                .build()
        );
        
        // Phase 2: Shoot 3 balls
        shootThreeBalls();
        
        // Phase 3: Move to pickup position
        telemetry.addData("Status", "Phase 3: Moving to pickup position");
        telemetry.update();
        
        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(-36, -24, Math.toRadians(90)))
                .splineTo(new Vector2d(0, -24), Math.toRadians(0))
                .build()
        );
        
        // Phase 4: Pickup balls
        pickupBalls();
        
        // Phase 5: Return to shooting position
        telemetry.addData("Status", "Phase 5: Returning to shooting position");
        telemetry.update();
        
        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(0, -24, Math.toRadians(0)))
                .splineTo(new Vector2d(24, -24), Math.toRadians(-90))
                .build()
        );
        
        // Phase 6: Final shooting
        shootThreeBalls();
    }
    
    private void shootThreeBalls() {
        telemetry.addData("Status", "Shooting 3 balls");
        telemetry.update();
        
        // Start flywheel
        setFlywheelRPM(3000);
        sleep(1000); // Wait for spinup
        
        // Intake and shoot sequence
        setIntakePower(1.0);
        sleep(500);
        
        for (int i = 0; i < 3; i++) {
            // Reverse pulse to shoot
            setIntakePower(-1.0);
            sleep(200);
            setIntakePower(1.0);
            sleep(500);
        }
        
        setIntakePower(0);
        setFlywheelRPM(0);
    }
    
    private void pickupBalls() {
        telemetry.addData("Status", "Picking up balls");
        telemetry.update();
        
        setIntakePower(1.0);
        sleep(2000); // Run intake for 2 seconds
        setIntakePower(0);
    }
    
    // Helper methods for hardware control
    private void setIntakePower(double power) {
        if (intake != null) {
            intake.setPower(power);
        }
    }
    
    private void setFlywheelRPM(double targetRPM) {
        if (flywheel != null) {
            // Convert RPM to encoder ticks per second
            double ticksPerSecond = (targetRPM * 28.0) / 60.0; // Assuming 28 CPR motor
            flywheel.setVelocity(ticksPerSecond);
        }
    }
    
    private double getFlywheelRPM() {
        if (flywheel != null) {
            double ticksPerSecond = flywheel.getVelocity();
            return (ticksPerSecond * 60.0) / 28.0; // Convert back to RPM
        }
        return 0;
    }
}
