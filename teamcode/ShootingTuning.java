package org.firstinspires.ftc.teamcode;

/**
 * Shooting tuning constants for the flywheel and intake system
 */
public class ShootingTuning {
    // Flywheel constants
    public static final double TARGET_FLYWHEEL_RPM = 3000.0;  // Target RPM for shooting
    public static final double MAX_RPM = 6000.0;              // Maximum motor RPM
    public static final double RPM_THRESHOLD = 2800.0;         // Minimum RPM to start shooting
    public static final double RPM_CHANGE_THRESHOLD = 200.0;   // RPM change to detect ball shot
    
    // Intake constants
    public static final double INTAKE_POWER = 1.0;            // Normal intake power
    public static final double REVERSE_INTAKE_POWER = -1.0;   // Reverse intake power for shooting
    
    // Timing constants
    public static final double INITIAL_INTAKE_TIME = 1.0;     // Time to run intake initially (seconds)
    public static final double INTAKE_REVERSE_SHORT = 0.1;   // Short reverse pulse time (seconds)
    public static final double INTAKE_REVERSE_LONG = 0.3;    // Long reverse pulse time (seconds)
    public static final double MAX_SHOOTING_TIME = 5.0;      // Maximum time for shooting sequence
    
    // Shake constants for clearing jams
    public static final double SHAKE_POWER = 0.3;            // Power for shaking robot
    public static final double SHAKE_DURATION = 0.2;         // Duration of each shake movement (seconds)
    public static final int SHAKE_CYCLES = 3;                 // Number of shake cycles
}
