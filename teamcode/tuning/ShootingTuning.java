package org.firstinspires.ftc.teamcode;

/**
 * Tuning parameters for the shooting sequence
 * All values are in seconds unless otherwise specified
 */
public class ShootingTuning {
    // Shooting sequence timing
    public static final double INITIAL_INTAKE_TIME = 2.0;       // Time to run intake before starting flywheel
    public static final double TARGET_FLYWHEEL_RPM = 1450.0;    // Target RPM for the flywheel
    public static final double RPM_THRESHOLD = 1425.0;           // Minimum RPM to consider flywheel at speed
    public static final double RPM_CHANGE_THRESHOLD = 100.0;     // RPM change to detect ball release
    public static final double MAX_RPM = 6000.0;                 // Max RPM for motor
    
    // Intake control during shooting
    public static final double INTAKE_REVERSE_SHORT = 0.2;       // Short reverse pulse time
    public static final double INTAKE_REVERSE_LONG = 1.0;        // Long reverse pulse time
    public static final double INTAKE_POWER = 1.0;               // Normal intake power
    public static final double REVERSE_INTAKE_POWER = -1.0;      // Reverse intake power
    
    // Shake parameters (if ball gets stuck)
    public static final int SHAKE_CYCLES = 4;                    // Number of up/down movements
    public static final double SHAKE_POWER = 0.8;                // Power for shake movement
    public static final double SHAKE_DURATION = 0.15;            // Duration of each shake movement
    
    // Timeouts
    public static final double MAX_SHOOTING_TIME = 10.0;         // Max time to attempt shooting sequence
    public static final double RPM_STABILIZE_TIME = 0.1;         // Time between RPM checks
    
    // Alignment parameters
    public static final double ALIGNMENT_ANGLE = 45.0;           // Degrees to face the goal
    public static final double ALIGNMENT_TOLERANCE = 2.0;        // Degrees tolerance for alignment
    
    // Flywheel PID constants
    public static final double FLYWHEEL_KP = 0.01;
    public static final double FLYWHEEL_KI = 0.0;
    public static final double FLYWHEEL_KD = 0.0;
    
    // Conversion factors
    public static final double NANOSECONDS_PER_SECOND = 1_000_000_000.0;
}
