package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Pedro Turret")
public class PedroTurret extends OpMode {

    private Follower follower;
    private Servo turret;

    private static final double TARGET_X = 136;
    private static final double TARGET_Y = 136;
    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_RANGE_RADIANS = Math.PI;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        turret = hardwareMap.get(Servo.class, "turret");

        // Optional: starting pose if you want
        follower.setPose(new Pose(135, 9, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        follower.update();

        Pose robotPose = follower.getPose();

        double dx = TARGET_X - robotPose.getX();
        double dy = TARGET_Y - robotPose.getY();

        double targetAngleField = Math.atan2(dy, dx);

        double turretYaw = angleWrap(targetAngleField - robotPose.getHeading());

        double servoPos = SERVO_CENTER + (turretYaw / SERVO_RANGE_RADIANS) ;

        servoPos = clamp(servoPos, 0.0, 1.0);

        turret.setPosition(servoPos);

        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Turret Yaw (deg)", Math.toDegrees(turretYaw));
        telemetry.addData("Servo Pos", servoPos);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
