package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class TurretPID1 {

    private Servo turret;
    private Webcam webcam;
    private Telemetry telemetry;

    // ---------------- PID ----------------
    private final double Kp = 0.025;
    private final double Ki = 0.0;
    private final double Kd = 0.004;

    private double integral = 0;
    private double lastError = 0;

    // Our physical state estimate (servo has no encoder)
    private double turretAngleEstimate = 0;

    // ---------------- Geometry ----------------
    private double targetAngle = 0;

    private double targetX = 0;
    private double targetY = 0;
    private boolean hasTarget = false;

    // ---------------- Servo limits ----------------
    private final double MIN_ANGLE = -90;
    private final double MAX_ANGLE = 90;

    // 👇 You tune these
    private final double SERVO_CENTER = 0.52;   // mechanical zero
    private final double SERVO_RANGE  = 0.42;   // total usable travel

    private final ElapsedTime timer = new ElapsedTime();

    // -----------------------------------------------------

    public void init(HardwareMap hw, Webcam webcam, Telemetry telemetry) {
        this.webcam = webcam;
        this.telemetry = telemetry;

        turret = hw.get(Servo.class, "turret");

        turretAngleEstimate = 0;
        lastError = 0;
        integral = 0;

        timer.reset();
        applyServo(turretAngleEstimate);
    }

    // Allows driver or auto to force a field target
    public void setManualTarget(double fieldX, double fieldY) {
        targetX = fieldX;
        targetY = fieldY;
        hasTarget = true;
    }

    // --- NEW: Reset integrator safely ---
    public void resetIntegrator() {
        integral = 0;
    }

    // --- NEW: Return the estimated turret angle for telemetry ---
    public double getEstimatedAngle() {
        return turretAngleEstimate;
    }

    private AprilTagDetection getBestTag() {
        List<AprilTagDetection> list = webcam.getDetectedTags();
        if (list.isEmpty()) return null;
        return list.get(0);
    }

    public void update(double robotX, double robotY, double robotHeadingDeg) {

        webcam.update();
        AprilTagDetection tag = getBestTag();

        // -------- Camera → Field coordinate transform --------
        if (tag != null) {
            double headingRad = Math.toRadians(robotHeadingDeg);

            double relX = tag.ftcPose.x;
            double relY = tag.ftcPose.y;

            double fieldX = relX * Math.cos(headingRad) - relY * Math.sin(headingRad);
            double fieldY = relX * Math.sin(headingRad) + relY * Math.cos(headingRad);

            targetX = robotX + fieldX;
            targetY = robotY + fieldY;
            hasTarget = true;
        }

        if (!hasTarget) return;

        // -------- Desired turret angle --------
        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double absoluteAngle = Math.toDegrees(Math.atan2(dy, dx));
        targetAngle = normalizeAngle(absoluteAngle - robotHeadingDeg);

        // -------- PID control --------
        double dt = timer.seconds();
        timer.reset();
        dt = Math.max(dt, 0.001);   // safety to prevent explosion

        double error = normalizeAngle(targetAngle - turretAngleEstimate);

        integral += error * dt;
        double dError = (error - lastError) / dt;

        double output = Kp * error + Ki * integral + Kd * dError;

        lastError = error;

        turretAngleEstimate += output * dt;
        turretAngleEstimate = clamp(turretAngleEstimate, MIN_ANGLE, MAX_ANGLE);

        applyServo(turretAngleEstimate);

        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Turret Estimate", turretAngleEstimate);
        telemetry.addData("Error", error);
    }

    // ---------------- Helpers ----------------

    private void applyServo(double angle) {
        double normalized = angle / 90.0;
        double pos = SERVO_CENTER + normalized * SERVO_RANGE;
        turret.setPosition(clamp(pos, 0, 1));
    }

    private double normalizeAngle(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
