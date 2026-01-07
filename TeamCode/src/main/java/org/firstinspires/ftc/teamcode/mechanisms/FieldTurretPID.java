package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class FieldTurretPID {

    private Servo turretServo;
    private Webcam webcam;
    private Telemetry telemetry;

    // PID constants (tune these)
    private final double Kp = 0.02;
    private final double Ki = 0.0;
    private final double Kd = 0.005;

    private double integral = 0;
    private double lastError = 0;

    // Turret servo limits
    private final double MIN_POS = 0.05;
    private final double MAX_POS = 0.95;

    // Servo angle mapping
    private final double DEGREES_TO_SERVO = (MAX_POS - MIN_POS) / 180.0;

    private double currentAngle = 0;
    private double targetAngle = 0;

    private ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hardwareMap, Webcam webcam, Telemetry telemetry) {
        this.webcam = webcam;
        this.telemetry = telemetry;
        turretServo = hardwareMap.get(Servo.class, "turret");
        setAngle(0); // center at start
    }

    /**
     * Update turret.
     * @param robotX Robot X position from PedroPathing (inches)
     * @param robotY Robot Y position from PedroPathing (inches)
     * @param robotHeading Robot heading from PedroPathing (degrees)
     * @param turretHeading Turret rotation relative to robot (degrees)
     */
    public void update(double robotX, double robotY, double robotHeading, double turretHeading) {
        // Update camera detections
        webcam.update();

        AprilTagDetection tag = getBestTag();
        if (tag != null) {
            // --- Field-Centric Calculation ---
            double tagX = tag.ftcPose.x; // field X in inches
            double tagY = tag.ftcPose.y; // field Y in inches

            double dx = tagX - robotX;
            double dy = tagY - robotY;

            double absoluteAngle = Math.toDegrees(Math.atan2(dy, dx)); // absolute field angle to tag

            // Relative to robot
            double robotRelative = absoluteAngle - robotHeading;

            // Correct for turret rotation (camera on turret)
            targetAngle = robotRelative - turretHeading;

            // Normalize to [-180,180]
            targetAngle = normalizeAngle(targetAngle);

            telemetry.addData("Target Angle", targetAngle);
        }

        // --- PID control ---
        double dt = timer.seconds();
        timer.reset();

        double error = normalizeAngle(targetAngle - currentAngle);
        integral += error * dt;
        double derivative = error / dt;

        double output = Kp * error + Ki * integral + Kd * derivative;

        currentAngle += output; // smooth PID step
        lastError = error;

        // Clamp and set servo
        setAngle(currentAngle);
    }

    private void setAngle(double angle) {
        angle = clamp(angle, -90, 90);
        double pos = 0.5 + angle * DEGREES_TO_SERVO;
        pos = clamp(pos, MIN_POS, MAX_POS);
        turretServo.setPosition(pos);
    }

    private AprilTagDetection getBestTag() {
        List<AprilTagDetection> detections = webcam.getDetectedTags();
        if (detections.isEmpty()) return null;
        return detections.get(0);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
