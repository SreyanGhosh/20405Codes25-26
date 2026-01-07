package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.mechanisms.TurretPID1;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "PedroTurretTeleOp", group = "TeleOp")
public class PedroTurretTeleOp extends OpMode {

    private Webcam webcam;
    private TurretPID1 turret;
    private Follower follower;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {

        webcam = new Webcam();
        webcam.init(hardwareMap, telemetry);

        turret = new TurretPID1();
        turret.init(hardwareMap, webcam, telemetry);

        follower = Constants.createFollower(hardwareMap);

        // Safe default start pose
        follower.setStartingPose(new Pose(88, 135, Math.toRadians(180)));

        telemetry.addLine("Turret system ready.");
    }

    @Override
    public void loop() {

        // --- Button edge detection ---
        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        // --- Reset localization ---
        if (aPressed) {
            follower.setStartingPose(new Pose(88, 135, Math.toRadians(180)));
            turret.resetIntegrator();   // prevents jump after reset
        }

        // --- Optional manual target ---
        if (bPressed) {
            turret.setManualTarget(140, 140);
        }

        // --- Update odometry ---
        follower.update();
        Pose pose = follower.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        // --- Main turret update ---
        turret.update(robotX, robotY, robotHeadingDeg);

        // --- Telemetry ---
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Heading", robotHeadingDeg);
        telemetry.addData("Turret Angle", turret.getEstimatedAngle());
        telemetry.update();
    }

    @Override
    public void stop() {
        webcam.stop();
    }
}
