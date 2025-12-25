package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTag Smooth 3D Tracker")
public class AprilTagSmooth3DTracker extends OpMode {

    // Vision
    private Webcam aprilTagWebcam = new Webcam();

    // Servo
    private Servo cameraServo;

    // Servo state
    private double servoPos = 0.5;

    // ===== TUNING CONSTANTS =====
    private static final int TARGET_TAG_ID = 24;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    // Gain: How aggressively to move toward the error
    private static double kP = 0.005;

    // Smoothness: Maximum change in servo position per loop cycle
    // Increase this (e.g., 0.005) for faster tracking, decrease for smoother cinematic movement
    private static final double MAX_STEP = 0.002;

    // Ignore tiny errors (Radians)
    private static final double DEADBAND_RAD = Math.toRadians(1.5);

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(servoPos);

        telemetry.addLine("Ready to track Tag 24 using 3D coordinates");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

        if (gamepad1.dpad_up) {
            kP += 0.0001;
        } else if (gamepad1.dpad_down) {
            kP -= 0.0001;
        }

        if (tag != null) {
            // Your robust math: Aiming at a point in 3D space
            // This works even when the tag is rotated or the robot strafes
            double x = tag.ftcPose.x;
            double z = tag.ftcPose.z;
            double yawError = Math.atan2(x, z);

            if (Math.abs(yawError) > DEADBAND_RAD) {
                double desiredDelta = yawError * kP;
                double clampedDelta = clamp(desiredDelta, -MAX_STEP, MAX_STEP);
                servoPos += clampedDelta;
            }


            // Keep within physical servo limits
            servoPos = clamp(servoPos, SERVO_MIN, SERVO_MAX);
            cameraServo.setPosition(servoPos);

            telemetry.addLine("STATUS: Tag Locked");
            telemetry.addData("Yaw Error (deg)", Math.toDegrees(yawError));
            telemetry.addData("Servo Pos", "%.3f", servoPos);

        } else {
            // Tag lost: Maintain last position smoothly
            cameraServo.setPosition(servoPos);
            telemetry.addLine("STATUS: Tag Lost - Holding last position");
        }
        telemetry.addData("kP", kP);
        telemetry.update();
    }

    // Helper to keep values in range
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}