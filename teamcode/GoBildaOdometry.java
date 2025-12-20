package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class GoBildaOdometry {
    // Constants for GoBilda PinPoint
    public static final double WHEEL_DIAMETER = 1.38; // inches (35mm)
    public static final double TICKS_PER_REV = 8192.0;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);

    // Calibration state
    private static boolean isCalibrated = false;
    private static double calibratedTrackWidth = 0;
    private static double calibratedWheelbase = 0;

    /**
     * Calibrates the odometry system by measuring the actual track width and wheelbase
     * based on encoder movements during a 360-degree turn.
     * 
     * @param leftEncoder Left odometry wheel encoder
     * @param rightEncoder Right odometry wheel encoder
     * @param initialTrackWidth Initial estimate of track width in inches
     * @param initialWheelbase Initial estimate of wheelbase in inches
     * @return Array containing [trackWidth, wheelbase] in inches
     */
    public static double[] calibrate(
            DcMotorEx leftEncoder,
            DcMotorEx rightEncoder,
            double initialTrackWidth,
            double initialWheelbase) {
        
        int[] lastLeftPos = new int[1];
        int[] lastRightPos = new int[1];
        double[] currentPose = new double[3]; // x, y, heading
        
        // Reset encoders
        lastLeftPos[0] = leftEncoder.getCurrentPosition();
        lastRightPos[0] = rightEncoder.getCurrentPosition();
        
        // Use the updateOdometry method to track the turn
        // This assumes the robot will be manually turned 360 degrees
        double initialHeading = 0;
        double finalHeading = 0;
        int sampleCount = 0;
        
        // Sample the encoders over time to calculate the actual turn
        // In practice, you would call this during a controlled turn
        while (sampleCount < 100) { // Sample for a reasonable amount of time
            double[] newPose = updateOdometry(
                leftEncoder, 
                rightEncoder, 
                lastLeftPos, 
                lastRightPos, 
                currentPose, 
                initialTrackWidth
            );
            
            System.arraycopy(newPose, 0, currentPose, 0, 3);
            
            if (sampleCount == 0) {
                initialHeading = currentPose[2];
            }
            
            finalHeading = currentPose[2];
            sampleCount++;
            
            // Small delay to prevent tight loop
            try { Thread.sleep(10); } catch (InterruptedException e) { break; }
        }
        
        // Calculate actual rotation (normalized to 0-2π)
        double totalRotation = (finalHeading - initialHeading) % (2 * Math.PI);
        if (totalRotation < 0) totalRotation += 2 * Math.PI;
        
        // Calculate actual distances moved
        double leftDistance = (leftEncoder.getCurrentPosition() - lastLeftPos[0]) / TICKS_PER_INCH;
        double rightDistance = (rightEncoder.getCurrentPosition() - lastRightPos[0]) / TICKS_PER_INCH;
        
        // Calculate actual track width
        if (Math.abs(totalRotation) > 0.1) { // If we've rotated a significant amount
            calibratedTrackWidth = Math.abs((leftDistance + rightDistance) / totalRotation);
            calibratedWheelbase = initialWheelbase; // Wheelbase is harder to calibrate automatically
        } else {
            calibratedTrackWidth = initialTrackWidth;
            calibratedWheelbase = initialWheelbase;
        }
        
        isCalibrated = true;
        return new double[]{calibratedTrackWidth, calibratedWheelbase};
    }

    /**
     * Updates the robot's position and heading based on encoder readings
     * Uses calibrated values if available, falls back to provided track width
     */
    public static double[] updateOdometry(
            DcMotorEx leftEncoder,
            DcMotorEx rightEncoder,
            int[] lastLeftPos,
            int[] lastRightPos,
            double[] currentPose,
            double trackWidth) {

        // Use calibrated track width if available
        double effectiveTrackWidth = isCalibrated ? calibratedTrackWidth : trackWidth;

        // Get current encoder positions
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();

        // Calculate delta movements
        int deltaLeft = leftPos - lastLeftPos[0];
        int deltaRight = rightPos - lastRightPos[0];

        // Update previous positions
        lastLeftPos[0] = leftPos;
        lastRightPos[0] = rightPos;

        // Convert ticks to inches
        double leftInches = deltaLeft / TICKS_PER_INCH;
        double rightInches = deltaRight / TICKS_PER_INCH;

        // Calculate change in heading (radians)
        double deltaHeading = (rightInches - leftInches) / effectiveTrackWidth;

        // Update heading (normalize to -π to π)
        double newHeading = (currentPose[2] + deltaHeading) % (2 * Math.PI);
        if (newHeading > Math.PI) newHeading -= 2 * Math.PI;
        if (newHeading < -Math.PI) newHeading += 2 * Math.PI;

        // Calculate change in position (average of both wheels)
        double deltaDistance = (leftInches + rightInches) / 2.0;

        // Update position
        double newX = currentPose[0] + deltaDistance * Math.cos(newHeading);
        double newY = currentPose[1] + deltaDistance * Math.sin(newHeading);

        return new double[]{newX, newY, newHeading};
    }

    /**
     * Resets the odometry tracking
     */
    public static double[] resetOdometry(
            DcMotorEx leftEncoder,
            DcMotorEx rightEncoder,
            int[] lastLeftPos,
            int[] lastRightPos) {

        if (leftEncoder != null && rightEncoder != null) {
            lastLeftPos[0] = leftEncoder.getCurrentPosition();
            lastRightPos[0] = rightEncoder.getCurrentPosition();
        }
        return new double[]{0, 0, 0}; // x, y, heading
    }

    /**
     * @return true if the odometry has been calibrated
     */
    public static boolean isCalibrated() {
        return isCalibrated;
    }

    /**
     * @return The calibrated track width in inches, or 0 if not calibrated
     */
    public static double getCalibratedTrackWidth() {
        return calibratedTrackWidth;
    }

    /**
     * @return The calibrated wheelbase in inches, or 0 if not calibrated
     */
    public static double getCalibratedWheelbase() {
        return calibratedWheelbase;
    }
}