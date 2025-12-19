package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Shooter Tuner", group = "ZZZ")
public class ShooterTuner extends LinearOpMode {
    // Hardware devices
    private DcMotorEx shooter;
    
    // Tuning parameters
    private double targetRPM = 1450.0;  // Target RPM for the shooter
    private double currentRPM = 0.0;    // Current measured RPM
    private double lastPosition = 0;    // For RPM calculation
    private long lastRpmTime = 0;       // For RPM calculation
    
    // Motor specifications
    private static final int TICKS_PER_REV = 28;  // Swyft 1:1 motor
    private static final double MAX_RPM = 1600.0;  // Updated max RPM
    
    // State
    private boolean isShooting = false;
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    
    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();
        
        while (opModeIsActive()) {
            updateRPM();
            updateControllerInput();
            updateTelemetry();
            sleep(20);
        }
        
        stopShooter();
    }
    
    private void initializeHardware() {
        try {
            // Initialize shooter motor
            shooter = hardwareMap.get(DcMotorEx.class, "shooter");
            
            if (shooter != null) {
                // Configure motor
                shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                
                // Reset RPM calculation variables
                lastPosition = shooter.getCurrentPosition();
                lastRpmTime = System.nanoTime();
                
                telemetry.addData("Status", "Shooter Initialized");
            } else {
                telemetry.addData("Error", "Shooter motor not found in hardware map");
            }
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize hardware: " + e.getMessage());
            telemetry.update();
            throw e;
        }
    }
    
    private void updateRPM() {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastRpmTime) / 1e9; // Convert to seconds
        
        if (deltaTime > 0.1) { // Update RPM at 10Hz
            int currentPosition = shooter.getCurrentPosition();
            double deltaPos = currentPosition - lastPosition;
            currentRPM = (deltaPos / deltaTime) * 60.0 / TICKS_PER_REV;
            
            lastPosition = currentPosition;
            lastRpmTime = currentTime;
        }
    }
    
    private void updateControllerInput() {
        try {
            // Toggle shooter on/off with A button
            if (gamepad1.a && !lastA) {
                if (isShooting) {
                    stopShooter();
                } else {
                    startShooter();
                }
            }
            
            // Adjust target RPM with D-pad (10 RPM steps)
            if (gamepad1.dpad_up && !lastDpadUp) {
                targetRPM = Math.min(MAX_RPM, targetRPM + 10);
            } else if (gamepad1.dpad_down && !lastDpadDown) {
                targetRPM = Math.max(0, targetRPM - 10);
            }
            
            // Reset RPM calculation on X button
            if (gamepad1.x && !lastX) {
                if (shooter != null) {
                    lastPosition = shooter.getCurrentPosition();
                    lastRpmTime = System.nanoTime();
                }
            }
            
            // Update button states
            lastA = gamepad1.a;
            lastX = gamepad1.x;
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            
        } catch (Exception e) {
            telemetry.addData("Controller Error", e.getMessage());
        }
    }
    
    private void startShooter() {
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double ticksPerSecond = (targetRPM / 60.0) * TICKS_PER_REV;
        shooter.setVelocity(ticksPerSecond);
        isShooting = true;
    }
    
    private void stopShooter() {
        if (shooter != null) {
            try {
                shooter.setPower(0);
                isShooting = false;
                currentRPM = 0;
            } catch (Exception e) {
                telemetry.addData("Stop Shooter Error", e.getMessage());
            }
        }
    }
    
    private void updateTelemetry() {
        telemetry.addLine("=== Shooter Tuner ===");
        telemetry.addData("Status", isShooting ? "RUNNING" : "STOPPED");
        telemetry.addData("Current RPM", "%.1f", currentRPM);
        telemetry.addData("Target RPM", "%.1f", targetRPM);
        telemetry.addData("Error", "%.1f RPM", targetRPM - currentRPM);
        telemetry.addLine("");
        telemetry.addData("Motor Power", "%.2f", shooter.getPower());
        telemetry.addData("Velocity", "%.1f ticks/sec", shooter.getVelocity());
        telemetry.addLine("");
        telemetry.addLine("=== Controls ===");
        telemetry.addLine("A: Toggle Shooter");
        telemetry.addLine("Dpad Up/Down: Adjust RPM");
        telemetry.addLine("X: Reset RPM Calc");
        telemetry.update();
    }
    
    private void saveConfiguration() {
        // This is a placeholder for saving configuration
        // In a real implementation, you would save these values to a file or configuration
        telemetry.addLine("Configuration Saved!");
        telemetry.addData("Target RPM", "%.1f", targetRPM);
        telemetry.update();
        sleep(500); // Show the message briefly
    }
}