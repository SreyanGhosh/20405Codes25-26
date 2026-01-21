package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;

public class TurretSubsystem {

    private final Servo turret;

    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_RANGE_RAD = 3.0979594;

    public TurretSubsystem(HardwareMap hw) {
        turret = hw.get(Servo.class, "turret");
    }

    public void aimAt(Pose robotPose, double targetX, double targetY) {
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        double fieldAngle = Math.atan2(dy, dx);
        double yaw = angleWrap(fieldAngle - robotPose.getHeading());

        double servoPos = SERVO_CENTER + yaw / SERVO_RANGE_RAD;
        turret.setPosition(clamp(servoPos, 0, 1));
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
