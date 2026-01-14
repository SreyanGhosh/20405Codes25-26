package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.PedroAuton1;
import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name="PedroTurretTeleOp", group="Test")
public class PedroTurretTeleOp extends OpMode {

    private Servo turret;
    private Webcam webcam;
    private Follower follower;

    // Exact goal coordinates (DECODE Season) in inches
    private static final double BLUE_GOAL_X = -58.37;
    private static final double BLUE_GOAL_Y = -55.64;
    private static final double RED_GOAL_X  = -58.37;
    private static final double RED_GOAL_Y  = 55.64;

    // AprilTag IDs
    private static final int BLUE_GOAL_TAG = 20;
    private static final int RED_GOAL_TAG  = 24;

    // Turret control
    private static final double kP = 0.9;
    private static final double MAX_STEP = 0.01;
    private static final double DEADBAND = Math.toRadians(1.5);
    private static final double VISION_WEIGHT = 0.35;

    private double turretAngle = 0;

    @Override
    public void init() {

        turret = hardwareMap.servo.get("turret");
        turret.setDirection(Servo.Direction.REVERSE);

        webcam = new Webcam();
        webcam.init(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 135, Math.toRadians(180)));

        telemetry.addLine("Exact Goal Hybrid Turret Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        follower.update();
        webcam.update();

        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        // Choose which alliance you're aiming at
        double goalX = BLUE_GOAL_X;
        double goalY = BLUE_GOAL_Y;
        int goalTagId = BLUE_GOAL_TAG;

        // (Replace with logic if you switch alliances)
        // Example: if(usingRedAlliance) { goalX = RED_GOAL_X; goalY = RED_GOAL_Y; goalTagId = RED_GOAL_TAG;}

        // Base geometric aim
        double fieldAngle = Math.atan2(goalY - robotY, goalX - robotX);
        double desiredTurretAngle = angleWrap(fieldAngle - robotHeading);

        // Vision correction (optional)
        AprilTagDetection tag = webcam.getTagBySpecificId(goalTagId);
        if (tag != null) {
            double visionError = Math.atan2(tag.ftcPose.x, tag.ftcPose.z);
            desiredTurretAngle += VISION_WEIGHT * visionError;
        }

        // Servo control
        double error = angleWrap(desiredTurretAngle - turretAngle);

        if (Math.abs(error) > DEADBAND) {
            double step = clamp(kP * error, -MAX_STEP, MAX_STEP);
            turretAngle = angleWrap(turretAngle + step);
            turret.setPosition(angleToServo(turretAngle));
        }

        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Heading (deg)", Math.toDegrees(robotHeading));
        telemetry.addData("Turret Angle (deg)", Math.toDegrees(turretAngle));
        telemetry.addData("Goal Visible", tag != null);
        telemetry.update();
    }

    private double angleWrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private double angleToServo(double a) {
        return clamp((a + Math.PI) / (2 * Math.PI), 0, 1);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}