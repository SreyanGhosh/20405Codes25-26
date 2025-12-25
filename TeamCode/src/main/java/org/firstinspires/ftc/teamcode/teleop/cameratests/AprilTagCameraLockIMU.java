package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "AprilTag Camera Lock (IMU Assist)")
public class AprilTagCameraLockIMU extends OpMode {

    private static final int TARGET_TAG_ID = 24;

    // Servo control (keep these conservative)
    private static final double SERVO_CENTER = 0.5;
    private static final double KP = 0.002;        // servo units per degree
    private static final double MAX_STEP = 0.01;
    private static final double DEADBAND_DEG = 0.8;

    // Bearing smoothing
    private static final double SMOOTH_ALPHA = 0.15;

    private Webcam aprilTagWebcam;
    private Servo cameraServo;
    private IMU imu;

    private double servoPos = SERVO_CENTER;
    private double smoothedBearing = 0.0;

    // IMU assist state
    private boolean haveFieldHeading = false;
    private double tagFieldHeadingDeg = 0.0;

    @Override
    public void init() {
        aprilTagWebcam = new Webcam();
        aprilTagWebcam.init(hardwareMap, telemetry);

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(servoPos);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        telemetry.addLine("AprilTag Camera Lock with IMU Assist Ready");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

        double robotYaw = getRobotYawDeg();
        double bearingDeg;

        if (tag != null) {
            // ===== VISION MODE =====
            smoothedBearing =
                    SMOOTH_ALPHA * tag.ftcPose.bearing +
                            (1.0 - SMOOTH_ALPHA) * smoothedBearing;

            bearingDeg = smoothedBearing;

            // Save absolute field heading of the tag
            tagFieldHeadingDeg = normalizeDeg(robotYaw + bearingDeg);
            haveFieldHeading = true;

            telemetry.addLine("VISION TRACKING");
        } else if (haveFieldHeading) {
            // ===== IMU ASSIST MODE =====
            bearingDeg = normalizeDeg(tagFieldHeadingDeg - robotYaw);

            telemetry.addLine("IMU ASSIST (Tag Lost)");
        } else {
            // No info at all yet
            cameraServo.setPosition(servoPos);
            telemetry.addLine("WAITING FOR TAG");
            telemetry.update();
            return;
        }

        // ===== SERVO CONTROL =====
        if (Math.abs(bearingDeg) > DEADBAND_DEG) {
            double correction = -KP * bearingDeg;
            correction = clamp(correction, -MAX_STEP, MAX_STEP);

            servoPos += correction;
            servoPos = clamp(servoPos, 0.0, 1.0);
        }

        cameraServo.setPosition(servoPos);

        telemetry.addData("Robot Yaw", "%.2f", robotYaw);
        telemetry.addData("Bearing Used", "%.2f", bearingDeg);
        telemetry.addData("Servo Pos", "%.3f", servoPos);
        telemetry.update();
    }

    // ================= HELPERS =================

    private double getRobotYawDeg() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    private double normalizeDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

