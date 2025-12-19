package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * PinpointOdometry - A more efficient implementation that uses GoBildaOdometry
 * for core odometry calculations while adding support for horizontal odometry.
 */
public class PinpointOdometry {
    // Odometry encoders
    private final DcMotorEx leftEncoder;
    private final DcMotorEx rightEncoder;
    private final DcMotorEx horizontalEncoder;
    
    // Track width and horizontal offset in inches
    private final double trackWidth;  // Distance between left and right odometry wheels
    private final double horizontalOffset;  // Distance from center to horizontal wheel
    
    // Position tracking
    private final double[] currentPose;  // [x, y, heading] in inches and radians
    private final int[] lastLeftPos = new int[1];
    private final int[] lastRightPos = new int[1];
    private int lastHorizontalPos;
    
    /**
     * Constructor for PinpointOdometry
     * 
     * @param hardwareMap The hardware map from the OpMode
     * @param leftEncoderName Name of the left encoder in the hardware map
     * @param rightEncoderName Name of the right encoder in the hardware map
     * @param horizontalEncoderName Name of the horizontal encoder in the hardware map
     * @param trackWidth Distance between left and right odometry wheels in inches
     * @param horizontalOffset Distance from center to horizontal wheel in inches
     */
    public PinpointOdometry(HardwareMap hardwareMap, 
                           String leftEncoderName, 
                           String rightEncoderName, 
                           String horizontalEncoderName,
                           double trackWidth,
                           double horizontalOffset) {
        
        // Initialize encoders
        this.leftEncoder = hardwareMap.get(DcMotorEx.class, leftEncoderName);
        this.rightEncoder = hardwareMap.get(DcMotorEx.class, rightEncoderName);
        this.horizontalEncoder = hardwareMap.get(DcMotorEx.class, horizontalEncoderName);
        
        // Set track width and horizontal offset
        this.trackWidth = trackWidth;
        this.horizontalOffset = horizontalOffset;
        
        // Initialize position tracking
        this.currentPose = new double[3]; // x, y, heading
        
        // Reset encoders and position
        resetPosition();
    }
    
    /**
     * Updates the robot's position and heading based on encoder readings
     */
    public void update() {
        // Update 2D odometry (x, y, heading) using GoBildaOdometry
        double[] newPose = GoBildaOdometry.updateOdometry(
            leftEncoder, 
            rightEncoder, 
            lastLeftPos, 
            lastRightPos, 
            currentPose,
            trackWidth
        );
        
        // Update current pose with new 2D odometry
        System.arraycopy(newPose, 0, currentPose, 0, 3);
        
        // Get current horizontal encoder position
        int horizontalPos = horizontalEncoder.getCurrentPosition();
        int deltaHorizontal = horizontalPos - lastHorizontalPos;
        lastHorizontalPos = horizontalPos;
        
        // Convert horizontal movement to field-relative coordinates
        if (deltaHorizontal != 0) {
            double horizontalInches = deltaHorizontal / GoBildaOdometry.TICKS_PER_INCH;
            double heading = currentPose[2];
            
            // Adjust position based on horizontal movement and current heading
            currentPose[0] += -horizontalInches * Math.sin(heading);
            currentPose[1] += horizontalInches * Math.cos(heading);
        }
    }
    
    /**
     * Resets the odometry system to the origin (0, 0, 0)
     */
    public void resetPosition() {
        // Reset position and heading
        currentPose[0] = 0;
        currentPose[1] = 0;
        currentPose[2] = 0;
        
        // Reset encoder positions
        if (leftEncoder != null) lastLeftPos[0] = leftEncoder.getCurrentPosition();
        if (rightEncoder != null) lastRightPos[0] = rightEncoder.getCurrentPosition();
        if (horizontalEncoder != null) lastHorizontalPos = horizontalEncoder.getCurrentPosition();
    }
    
    /**
     * @return Current X position in inches
     */
    public double getX() { 
        return currentPose[0]; 
    }
    
    /**
     * @return Current Y position in inches
     */
    public double getY() { 
        return currentPose[1]; 
    }
    
    /**
     * @return Current heading in radians
     */
    public double getHeadingRadians() { 
        return currentPose[2]; 
    }
    
    /**
     * @return Current heading in degrees
     */
    public double getHeading() { 
        return Math.toDegrees(currentPose[2]); 
    }
    
    /**
     * @return The current pose as an array [x, y, heading] where x and y are in inches and heading is in radians
     */
    public double[] getPose() {
        return currentPose.clone();
    }
}
