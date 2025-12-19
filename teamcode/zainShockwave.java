package org.firstinspires.ftc.teamcode; // Change if your package is different

// FTC SDK Imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



// TeleOp OpMode annotation
@TeleOp(name="DO NOT TOUCH", group="DECODE")
public class zainShockwave extends OpMode {

    // Hardware variables
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx flywheel;
    private DcMotor intake;
   // private Servo trapdoor;

    private ElapsedTime runtime = new ElapsedTime();

    private PinpointOdometry odometry; // Add this line to declare the odometry instance

    @Override
    public void init() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        // Initialize the odometry with hardware map and configuration
        // Update these values based on your robot's configuration:
        // - Encoder names in the hardware map
        // - Track width (distance between left and right wheels in inches)
        // - Horizontal offset (distance from center to horizontal wheel in inches)
        odometry = new PinpointOdometry(
            hardwareMap,           // Hardware map from the OpMode
            "left_encoder",        // Name of left encoder in config
            "right_encoder",       // Name of right encoder in config
            "horizontal_encoder",  // Name of horizontal encoder in config
            15.0,                  // Track width in inches (adjust based on your robot)
            6.0                    // Forward offset in inches (adjust based on your robot)
        );

        intake = hardwareMap.get(DcMotor.class, "intake");
       // trapdoor = hardwareMap.get(Servo.class, "trapdoor");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get the robot's heading from the pinpoint odometry
        double heading = odometry.getHeading(); // Use the odometry instance to get the heading


        // Calculate the drive direction based on the heading
        double drive = -gamepad1.left_stick_y; // Forward/backward
        double turn = gamepad1.right_stick_x; // Left/right rotation

        // Adjust drive direction for field-centric control
        double adjustedDrive = drive * Math.cos(heading) - turn * Math.sin(heading);
        double adjustedTurn = drive * Math.sin(heading) + turn * Math.cos(heading);

        double speedMultiplier = 1.0; // Default speed
        if (gamepad1.left_bumper) {
            speedMultiplier = 0.5; // Slow mode
        }

        // Set power to motors for mecanum wheels with speed multiplier
        leftFront.setPower(Range.clip((adjustedDrive + adjustedTurn) * speedMultiplier, -1.0, 1.0));
        leftBack.setPower(Range.clip((adjustedDrive - adjustedTurn) * speedMultiplier, -1.0, 1.0));
        rightFront.setPower(Range.clip((adjustedDrive - adjustedTurn) * speedMultiplier, -1.0, 1.0));
        rightBack.setPower(Range.clip((adjustedDrive + adjustedTurn) * speedMultiplier, -1.0, 1.0));

        // Intake control using triggers
        if (gamepad1.left_trigger > 0.5) {
            intake.setPower(1); // Intake on
        } else if (gamepad1.right_trigger > 0.5) {
            intake.setPower(-1); // Intake reverse
        } else {
            intake.setPower(0); // Intake off
        }
    }

    @Override
    public void stop() {
        // Optional cleanup
    }
}
