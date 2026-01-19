package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Flywheel Distance Shooter")
public class FlywheelDistanceShooter extends OpMode {

    // Hardware
    private DcMotorEx flywheel;
    private Webcam webcam = new Webcam();

    // State
    private boolean running = false;
    private double targetVelocity = 0;

    // PIDF (tuned for your robot)
    private static final double kP = 0.00215;
    private static final double kI = 0.0;
    private static final double kD = 0.0001;
    private static final double kF = 19.9; // measured for belt-driven 1:1 flywheel

    // Shooter limits
    private static final double MIN_VELOCITY = 200;
    private static final double MAX_VELOCITY = 3000;

    // Ready-to-shoot threshold
    private static final double VELOCITY_TOLERANCE = 10;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheel.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );

        webcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        // Flywheel ON/OFF
        if (gamepad1.right_bumper) running = true;
        if (gamepad1.left_bumper)  running = false;

        // Update AprilTag distance
        webcam.update();
        AprilTagDetection tag = webcam.getTagBySpecificId(24);

        if (tag != null) {
            // Distance is already in inches
            double distanceInches = tag.ftcPose.range;

            // Optional safety clamp
            distanceInches = clamp(distanceInches, 5, 200);

            // Compute target velocity from distance
            targetVelocity = distanceToVelocity(distanceInches);
        }

        // Apply velocity
        if (running && targetVelocity > 0) {
            flywheel.setVelocity(targetVelocity);
        } else {
            flywheel.setVelocity(0);
        }

        // Telemetry
        telemetry.addData("Flywheel", running ? "RUNNING" : "STOPPED");

        if (tag != null) {
            telemetry.addData("Distance (in)", tag.ftcPose.range);
            telemetry.addData("Target Velocity", targetVelocity);
        } else {
            telemetry.addLine("No AprilTag");
        }

        double actualVelocity = flywheel.getVelocity();
        telemetry.addData("Actual Velocity", actualVelocity);

        boolean ready = Math.abs(actualVelocity - targetVelocity) <= VELOCITY_TOLERANCE;
        telemetry.addData("Shooter Ready", ready ? "YES" : "NO");

        telemetry.update();
    }

    /**
     * Distance → Flywheel velocity using inverse of:
     * y = 0.0000344047x^2 - 0.0310522x + 10.23346
     * y = distance (in), x = velocity (ticks/sec)
     */
    private double distanceToVelocity(double distance) {
        double a = 0.0310522;
        double b = 0.0096424;
        double c = 0.0001376188;
        double d = 10.23346;
        double e = 0.0000688094;


        // LEFT branch for valid velocities
        double velocity = (a - Math.sqrt(b - c * (d - distance))) / e;

        // Clamp to safe range
        return clamp(velocity, MIN_VELOCITY, MAX_VELOCITY);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
