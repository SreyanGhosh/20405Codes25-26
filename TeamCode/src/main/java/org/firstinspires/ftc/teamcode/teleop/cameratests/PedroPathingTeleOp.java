package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "PedroPathingTeleOp")
public class PedroPathingTeleOp extends OpMode {

    private Servo turret;
    private Webcam webcam;
    private Follower follower;
    private ElapsedTime lastTagTimer = new ElapsedTime();

    private static final int TAG_ID = 22;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double kP = 0.8;
    private static final double MAX_SERVO_STEP = 0.008;
    private static final double ANGLE_DEADBAND = Math.toRadians(3.0);

    // AprilTag field coordinates (FTC SDK)
    private static final double TAG_FIELD_X = -58.3727; // backward
    private static final double TAG_FIELD_Y = 55.6425;  // right

    private double turretAngleRad = 0.0;
    private double targetAngleRad = 0.0;
    private double servoPos = 0.5;

    private static final double SWEEP_SPEED = 0.005;
    private boolean firstTagDetected = false;

    @Override
    public void init() {
        turret = hardwareMap.get(Servo.class, "turret");
        turret.setDirection(Servo.Direction.REVERSE);

        webcam = new Webcam();
        webcam.init(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);

        // Set auton end pose as initial Pedro pose estimate
        follower.setStartingPose(new Pose(88, 135, Math.toRadians(180)));

        turretAngleRad = 0.0;
        targetAngleRad = 0.0;
        servoPos = angleToServo(turretAngleRad);
        turret.setPosition(servoPos);

        telemetry.addLine("AprilTag Turret Vision-First Pedro Fixed Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        webcam.update();
        AprilTagDetection tag = webcam.getTagBySpecificId(TAG_ID);

        follower.update();
        Pose robotPose = follower.getPose();
        double robotHeading = robotPose.getHeading();

        boolean tagVisible = (tag != null && tag.metadata != null);

        if (tagVisible) {
            lastTagTimer.reset();

            // ---------------- Absolute Turret Target ----------------
            double rawAngle = Math.atan2(tag.ftcPose.x, tag.ftcPose.y); // relative to camera
            targetAngleRad = angleWrap(turretAngleRad + rawAngle);

            // ---------------- First Tag Detection ----------------
            if (!firstTagDetected) {
                firstTagDetected = true;
                telemetry.addLine("First AprilTag detected: snapping Pedro pose.");
            }

            // ---------------- Update Pedro Pose ----------------
            correctPedroPoseFromTag(tag, turretAngleRad, robotHeading);

        } else {
            if (!firstTagDetected) {
                // Sweep turret slowly until first tag is detected
                targetAngleRad += SWEEP_SPEED;
            }
            // After first tag detected, rely on Pedro Pathing
        }

        // ---------------- Turret Control ----------------
        double error = angleWrap(targetAngleRad - turretAngleRad);
        if (Math.abs(error) > ANGLE_DEADBAND) {
            double delta = clamp(kP * error, -MAX_SERVO_STEP, MAX_SERVO_STEP);
            turretAngleRad = angleWrap(turretAngleRad + delta);
            servoPos = angleToServo(turretAngleRad);
            turret.setPosition(servoPos);
            telemetry.addLine("Tracking Tag or Sweeping");
        } else {
            telemetry.addLine("Holding Position");
        }

        // ---------------- Telemetry ----------------
        telemetry.addData("Servo Pos", "%.3f", servoPos);
        telemetry.addData("Turret Angle (deg)", Math.toDegrees(turretAngleRad));
        telemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngleRad));
        telemetry.addData("Tag Visible", tagVisible);
        telemetry.addData("Pedro X", follower.getPose().getX());
        telemetry.addData("Pedro Y", follower.getPose().getY());
        telemetry.addData("Pedro Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("First Tag Detected", firstTagDetected);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPosition(servoPos);
        webcam.stop();
    }

    // ---------------- Pedro Pose Correction (fixed 72-inch conversion) ----------------
    private void correctPedroPoseFromTag(AprilTagDetection tag, double turretAngleRad, double robotHeading) {
        // FTC SDK horizontal plane: X = right, Y = forward
        double tagX = tag.ftcPose.x;
        double tagY = tag.ftcPose.y;

        double distance = Math.hypot(tagX, tagY);
        double angleCam = Math.atan2(tagX, tagY);

        // Absolute angle from robot to tag in field frame
        double thetaAbsolute = robotHeading + turretAngleRad + angleCam;

        // Robot position in FTC field coordinates
        double robotFieldX = TAG_FIELD_X - distance * Math.cos(thetaAbsolute);
        double robotFieldY = TAG_FIELD_Y - distance * Math.sin(thetaAbsolute);

        // Pedro Pathing coordinates: just linear offset by 72 inches, no axis swap
        double pedroX = 72 + robotFieldX;
        double pedroY = 72 + robotFieldY;

        follower.setPose(new Pose(pedroX, pedroY, robotHeading));
    }

    // ---------------- Math Utilities ----------------
    private double angleWrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private double angleToServo(double angleRad) {
        double normalized = (angleRad + Math.PI) / (2.0 * Math.PI);
        return clamp(normalized, SERVO_MIN, SERVO_MAX);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}