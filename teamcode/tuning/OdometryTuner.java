package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.PinpointOdometry;

/**
 * OdometryTuner - A comprehensive tuning utility for your odometry system.
 * 
 * This OpMode helps you tune various odometry parameters:
 * 1. Track width (distance between left and right odometry wheels)
 * 2. Forward offset (distance from center of rotation to forward odometry wheel)
 * 3. Ticks per inch calibration
 * 4. Odometry wheel diameter verification
 */
@TeleOp(name = "Odometry Tuner", group = "ZZZ")
public class OdometryTuner extends LinearOpMode {
    // Hardware devices
    private DcMotorEx leftEncoder;
    private DcMotorEx rightEncoder;
    private DcMotorEx horizontalEncoder;
    
    // Odometry systems
    private PinpointOdometry pinpointOdometry;
    private double[] gobildaPose = new double[3]; // x, y, heading
    private int[] lastLeftPos = new int[1];
    private int[] lastRightPos = new int[1];
    
    // Tuning parameters
    private double trackWidth = 15.0; // Initial guess in inches
    private double forwardOffset = 6.0; // Initial guess in inches
    private double wheelDiameter = GoBildaOdometry.WHEEL_DIAMETER; // Default from GoBildaOdometry
    
    // Tuning state
    private TuningMode mode = TuningMode.TRACK_WIDTH;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    
    // Timing
    private ElapsedTime timer = new ElapsedTime();
    private double lastUpdateTime = 0;
    
    // For calibration movements (commented out as not currently used)
    // private double startX = 0;
    // private double startY = 0;
    // private double startHeading = 0;
    
    private enum TuningMode {
        TRACK_WIDTH,
        FORWARD_OFFSET,
        STRAIGHT_TEST,
        TURN_TEST,
        COMPARISON
    }
    
    @Override
    public void runOpMode() {
        try {
            initializeHardware();
            telemetry.addData("Status", "Waiting for start...");
            telemetry.addLine("\nModes:");
            telemetry.addLine("A: Track Width Tuning");
            telemetry.addLine("B: Forward Offset Tuning");
            telemetry.addLine("X: Straight Line Test");
            telemetry.addLine("Y: Turn Test");
            telemetry.addLine("LB: Compare Systems");
            telemetry.update();
            
            waitForStart();
            
            while (opModeIsActive()) {
                try {
                    updateControllerInput();
                    updateOdometry();
                    
                    // Slow down the update rate to make it more manageable
                    double currentTime = timer.seconds();
                    if (currentTime - lastUpdateTime < 0.1) {
                        continue;
                    }
                    lastUpdateTime = currentTime;
                    
                    // Update odometry based on current mode
                    telemetry.clear();
                    switch (mode) {
                        case TRACK_WIDTH:
                            updateTrackWidthTuning();
                            break;
                        case FORWARD_OFFSET:
                            updateForwardOffsetTuning();
                            break;
                        case STRAIGHT_TEST:
                            updateStraightTest();
                            break;
                        case TURN_TEST:
                            updateTurnTest();
                            break;
                        case COMPARISON:
                            updateComparison();
                            break;
                    }
                } catch (Exception e) {
                    telemetry.addData("Runtime Error", e.getMessage());
                    telemetry.update();
                    sleep(1000); // Prevent log spam
                }
            }
        } catch (Exception e) {
            telemetry.addData("Fatal Error", e.getMessage());
            telemetry.update();
        } finally {
            // Clean up resources if needed
        }
    }
    
    private void initializeHardware() {
        try {
            // Initialize encoders - update these names to match your configuration
            leftEncoder = hardwareMap.get(DcMotorEx.class, "left_encoder");
            rightEncoder = hardwareMap.get(DcMotorEx.class, "right_encoder");
            horizontalEncoder = hardwareMap.get(DcMotorEx.class, "horizontal_encoder");
            
            // Configure encoders
            if (leftEncoder != null && rightEncoder != null && horizontalEncoder != null) {
                // Reset encoders
                leftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                horizontalEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                
                // Set to run without encoder
                leftEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                rightEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                horizontalEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                
                // Initialize odometry systems
                pinpointOdometry = new PinpointOdometry(
                    hardwareMap,
                    "left_encoder",
                    "right_encoder",
                    "horizontal_encoder",
                    trackWidth,
                    forwardOffset
                );
                
                // Initialize GoBilda odometry tracking
                lastLeftPos[0] = leftEncoder.getCurrentPosition();
                lastRightPos[0] = rightEncoder.getCurrentPosition();
                
                telemetry.addData("Status", "Hardware and Odometry Initialized");
            } else {
                if (leftEncoder == null) telemetry.addData("Error", "Left encoder not found");
                if (rightEncoder == null) telemetry.addData("Error", "Right encoder not found");
                if (horizontalEncoder == null) telemetry.addData("Error", "Horizontal encoder not found");
            }
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }
        telemetry.update();
    }
    
    private void updateControllerInput() {
        // Toggle modes
        if (gamepad1.a && !lastA) {
            mode = TuningMode.TRACK_WIDTH;
        } else if (gamepad1.b && !lastB) {
            mode = TuningMode.FORWARD_OFFSET;
        } else if (gamepad1.x && !lastX) {
            mode = TuningMode.STRAIGHT_TEST;
        } else if (gamepad1.y && !lastY) {
            mode = TuningMode.TURN_TEST;
        } else if (gamepad1.left_bumper && !lastLeftBumper) {
            mode = TuningMode.COMPARISON;
        }
        
        // Adjust parameters
        double adjustAmount = gamepad1.right_bumper ? 0.001 : 0.01;
        if (gamepad1.dpad_up) {
            if (mode == TuningMode.TRACK_WIDTH) {
                trackWidth += adjustAmount;
                updatePinpointOdometryConfig();
            } else if (mode == TuningMode.FORWARD_OFFSET) {
                forwardOffset += adjustAmount;
                updatePinpointOdometryConfig();
            }
        } else if (gamepad1.dpad_down) {
            if (mode == TuningMode.TRACK_WIDTH) {
                trackWidth = Math.max(0.1, trackWidth - adjustAmount);
                updatePinpointOdometryConfig();
            } else if (mode == TuningMode.FORWARD_OFFSET) {
                forwardOffset = Math.max(0.1, forwardOffset - adjustAmount);
                updatePinpointOdometryConfig();
            }
        }
        
        // Reset position on back button
        if (gamepad1.back) {
            resetOdometry();
        }
        
        // Save values
        if (gamepad1.start) {
            saveCalibration();
        }
        
        // Update button states
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
    }
    
    private void updateOdometry() {
        if (leftEncoder == null || rightEncoder == null) return;
        
        // Update GoBilda odometry
        gobildaPose = GoBildaOdometry.updateOdometry(
            leftEncoder,
            rightEncoder,
            lastLeftPos,
            lastRightPos,
            gobildaPose,
            trackWidth
        );
        
        // Pinpoint odometry is updated automatically in its own thread
    }
    
    private void updatePinpointOdometryConfig() {
        if (pinpointOdometry != null) {
            // Recreate with new parameters
            pinpointOdometry = new PinpointOdometry(
                hardwareMap,
                "left_encoder",
                "right_encoder",
                "horizontal_encoder",
                trackWidth,
                forwardOffset
            );
        }
    }
    
    private void resetOdometry() {
        if (leftEncoder != null && rightEncoder != null) {
            lastLeftPos[0] = leftEncoder.getCurrentPosition();
            lastRightPos[0] = rightEncoder.getCurrentPosition();
        }
        gobildaPose = new double[3]; // Reset to origin
        if (pinpointOdometry != null) {
            pinpointOdometry.resetPosition();
        }
    }
    
    private void updateTrackWidthTuning() {
        try {
            // Display instructions and current values
            telemetry.addLine("=== Track Width Tuning ===");
            telemetry.addLine("Drive in a straight line and measure the actual distance.");
            telemetry.addLine("Adjust until the reported distance matches the actual distance.");
            telemetry.addLine("");
            telemetry.addLine("Controls:");
            telemetry.addLine("Dpad Up/Down: Adjust track width");
            telemetry.addLine("A/B/X/Y: Switch modes | Start: Save | Back: Reset");
            telemetry.addLine("");
            telemetry.addData("Current Track Width", "%.4f inches", trackWidth);
            
            if (leftEncoder != null && rightEncoder != null) {
                // Show both odometry systems' positions
                telemetry.addLine("\nGoBilda Odometry:");
                telemetry.addData("  X", "%.2f inches", gobildaPose[0]);
                telemetry.addData("  Y", "%.2f inches", gobildaPose[1]);
                telemetry.addData("  Heading", "%.2f°", Math.toDegrees(gobildaPose[2]));
                
                if (pinpointOdometry != null) {
                    double[] pinpointPose = pinpointOdometry.getPose();
                    telemetry.addLine("\nPinpoint Odometry:");
                    telemetry.addData("  X", "%.2f inches", pinpointPose[0]);
                    telemetry.addData("  Y", "%.2f inches", pinpointPose[1]);
                    telemetry.addData("  Heading", "%.2f°", Math.toDegrees(pinpointPose[2]));
                }
                
                telemetry.addLine("\nEncoder Counts:");
                telemetry.addData("  Left", leftEncoder.getCurrentPosition());
                telemetry.addData("  Right", rightEncoder.getCurrentPosition());
                telemetry.addData("  Horizontal", horizontalEncoder != null ? 
                    horizontalEncoder.getCurrentPosition() : "N/A");
            } else {
                telemetry.addLine("Error: Required encoders not detected");
            }
        } catch (Exception e) {
            telemetry.addData("Track Width Error", e.getMessage());
        }
    }
    
    private void updateForwardOffsetTuning() {
        telemetry.addLine("=== Forward Offset Tuning ===");
        telemetry.addLine("Drive in an arc and measure the actual angle turned.");
        telemetry.addLine("Adjust until the reported angle matches the actual angle.");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("Dpad Up/Down: Adjust forward offset");
        telemetry.addLine("A/B/X/Y: Switch modes | Start: Save | Back: Reset");
        telemetry.addLine("");
        telemetry.addData("Current Forward Offset", "%.4f inches", forwardOffset);
        
        if (pinpointOdometry != null) {
            double[] pinpointPose = pinpointOdometry.getPose();
            telemetry.addLine("\nPinpoint Odometry:");
            telemetry.addData("  X", "%.2f inches", pinpointPose[0]);
            telemetry.addData("  Y", "%.2f inches", pinpointPose[1]);
            telemetry.addData("  Heading", "%.2f°", Math.toDegrees(pinpointPose[2]));
        }
        
        telemetry.addLine("\nGoBilda Odometry:");
        telemetry.addData("  X", "%.2f inches", gobildaPose[0]);
        telemetry.addData("  Y", "%.2f inches", gobildaPose[1]);
        telemetry.addData("  Heading", "%.2f°", Math.toDegrees(gobildaPose[2]));
    }
    
    private void updateStraightTest() {
        telemetry.addLine("=== Straight Line Test ===");
        telemetry.addLine("Drive in a straight line and verify the reported distance.");
        telemetry.addLine("");
        telemetry.addData("Left Encoder (ticks)", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder (ticks)", rightEncoder.getCurrentPosition());
        telemetry.addData("Horizontal Encoder (ticks)", horizontalEncoder.getCurrentPosition());
        telemetry.addData("Calculated Distance", "%.2f inches", 
            (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2.0 / 
            (GoBildaOdometry.TICKS_PER_REV / (wheelDiameter * Math.PI)));
    }
    
    private void updateTurnTest() {
        telemetry.addLine("=== Turn Test ===");
        telemetry.addLine("Turn the robot and verify the reported angle.");
        telemetry.addLine("");
        telemetry.addData("Left Encoder (ticks)", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder (ticks)", rightEncoder.getCurrentPosition());
        telemetry.addData("Calculated Angle", "%.2f degrees", 
            Math.toDegrees((rightEncoder.getCurrentPosition() - leftEncoder.getCurrentPosition()) / 
            (trackWidth * GoBildaOdometry.TICKS_PER_INCH)));
    }
    
    private void updateComparison() {
        telemetry.addLine("=== Odometry Comparison ===");
        telemetry.addLine("Compare both odometry systems side by side");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("A/B/X/Y: Switch modes | Back: Reset Position");
        
        if (pinpointOdometry != null) {
            double[] pinpointPose = pinpointOdometry.getPose();
            
            telemetry.addLine("\n=== Pinpoint Odometry ===");
            telemetry.addData("  X", "%.2f inches", pinpointPose[0]);
            telemetry.addData("  Y", "%.2f inches", pinpointPose[1]);
            telemetry.addData("  Heading", "%.2f°", Math.toDegrees(pinpointPose[2]));
            telemetry.addData("  Track Width", "%.3f in", trackWidth);
            telemetry.addData("  Fwd Offset", "%.3f in", forwardOffset);
        }
        
        telemetry.addLine("\n=== GoBilda Odometry ===");
        telemetry.addData("  X", "%.2f inches", gobildaPose[0]);
        telemetry.addData("  Y", "%.2f inches", gobildaPose[1]);
        telemetry.addData("  Heading", "%.2f°", Math.toDegrees(gobildaPose[2]));
        telemetry.addData("  Track Width", "%.3f in", trackWidth);
        telemetry.addData("  Ticks/Inch", "%.3f", GoBildaOdometry.TICKS_PER_INCH);
    }
    
    private void saveCalibration() {
        try {
            // In a real implementation, you would save these values to a file or configuration
            telemetry.addLine("Calibration values saved!");
            telemetry.addLine("");
            telemetry.addLine("Add these to your code:");
            telemetry.addData("Track Width", "%.4f", trackWidth);
            telemetry.addData("Forward Offset", "%.4f", forwardOffset);
            telemetry.addLine("");
            telemetry.addLine("PinpointOdometry odometry = new PinpointOdometry(");
            telemetry.addLine("    hardwareMap,");
            telemetry.addLine("    \"left_encoder\",");
            telemetry.addLine("    \"right_encoder\",");
            telemetry.addLine("    \"horizontal_encoder\",");
            telemetry.addData("    %.4f,  // Track width", trackWidth);
            telemetry.addData("    %.4f   // Forward offset", forwardOffset);
            telemetry.addLine(")");
            
            // Show the saved message for a moment
            telemetry.update();
            sleep(3000);
        } catch (Exception e) {
            telemetry.addData("Save Error", e.getMessage());
            telemetry.update();
        }
    }
    
    private void updateTelemetry() {
        // Display current mode
        telemetry.addData("Mode", mode.toString());
        telemetry.addLine("");
        
        // Display current encoder values
        telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
        telemetry.addData("Horizontal Encoder", horizontalEncoder.getCurrentPosition());
        
        // Display current parameters
        telemetry.addLine("");
        telemetry.addData("Track Width", "%.4f inches", trackWidth);
        telemetry.addData("Forward Offset", "%.4f inches", forwardOffset);
        telemetry.addData("Wheel Diameter", "%.4f inches", wheelDiameter);
        
        // Display instructions
        telemetry.addLine("");
        telemetry.addLine("A: Track Width  B: Forward Offset  X: Straight Test  Y: Turn Test");
        telemetry.addLine("Dpad: Adjust  Start: Save  Back: Reset Encoders");
    }
}
