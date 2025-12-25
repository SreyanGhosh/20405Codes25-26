package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTag Camera Lock (Smoothed)")
public class AprilTagCameraLockSmoothed extends OpMode {

    // ================= CONFIG =================
    private static final int TARGET_TAG_ID = 24;

    // Servo tuning
    private static final double SERVO_CENTER = 0.5;
    private static final double DEG_TO_SERVO = 1.0 / 90.0; // CALIBRATE
    private static final double DEADBAND_DEG = 1.0;

    // Bearing smoothing
    private static final int SMOOTHING_WINDOW = 5;

    // ==========================================
    private Webcam aprilTagWebcam;
    private Servo cameraServo;

    // Remember last servo position
    private double servoPos = SERVO_CENTER;

    // Bearing smoothing storage
    private double[] bearingHistory = new double[SMOOTHING_WINDOW];
    private int bearingIndex = 0;
    private int bearingCount = 0;

    @Override
    public void init() {
        aprilTagWebcam = new Webcam();
        aprilTagWebcam.init(hardwareMap, telemetry);

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(servoPos);

        telemetry.addLine("AprilTag Camera Lock with Smoothing Ready");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

        if (tag != null) {
            double bearingDeg = smoothBearing(tag.ftcPose.bearing);

            if (Math.abs(bearingDeg) > DEADBAND_DEG) {
                servoPos -= bearingDeg * DEG_TO_SERVO;
                servoPos = clamp(servoPos, 0.0, 1.0);
            }

            cameraServo.setPosition(servoPos);

            telemetry.addLine("Tag FOUND");
            telemetry.addData("Raw Bearing", "%.2f", tag.ftcPose.bearing);
            telemetry.addData("Smoothed Bearing", "%.2f", bearingDeg);
            telemetry.addData("Servo Pos", "%.3f", servoPos);
        } else {
            cameraServo.setPosition(servoPos);

            telemetry.addLine("Tag LOST – holding last position");
            telemetry.addData("Servo Pos", "%.3f", servoPos);
        }

        telemetry.update();
    }

    // ================= HELPERS =================

    private double smoothBearing(double newBearing) {
        bearingHistory[bearingIndex] = newBearing;
        bearingIndex = (bearingIndex + 1) % SMOOTHING_WINDOW;

        if (bearingCount < SMOOTHING_WINDOW) {
            bearingCount++;
        }

        double sum = 0.0;
        for (int i = 0; i < bearingCount; i++) {
            sum += bearingHistory[i];
        }
        return sum / bearingCount;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
