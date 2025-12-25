package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTag Camera Lock Hold")
public class AprilTagCameraLockHold extends OpMode {

    private static final int TARGET_TAG_ID = 24;

    // Servo tuning
    private static final double SERVO_CENTER = 0.5;
    private static final double DEG_TO_SERVO = 1.0 / 120.0;
    private static final double DEADBAND_DEG = 1;
    private static final double MAX_STEP = 0.001;

    private Webcam aprilTagWebcam;
    private Servo cameraServo;

    private double servoPos = SERVO_CENTER;

    @Override
    public void init() {
        aprilTagWebcam = new Webcam();
        aprilTagWebcam.init(hardwareMap, telemetry);

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(servoPos);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

        if (tag != null) {
            double bearingDeg = tag.ftcPose.bearing;

            if (Math.abs(bearingDeg) > DEADBAND_DEG) {
                double delta = -bearingDeg * DEG_TO_SERVO;
                delta = clamp(delta, -MAX_STEP, MAX_STEP);

                servoPos += delta;
                servoPos = clamp(servoPos, 0.0, 1.0);
            }

            cameraServo.setPosition(servoPos);

            telemetry.addLine("Tag FOUND");
            telemetry.addData("Bearing (deg)", "%.2f", bearingDeg);
            telemetry.addData("Servo Pos", "%.3f", servoPos);
        } else {
            cameraServo.setPosition(servoPos);
            telemetry.addLine("Tag LOST – holding");
        }

        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

