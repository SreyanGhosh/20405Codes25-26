package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTag Camera Lock")
public class AprilTagCameraLock extends OpMode {

    private static final int TARGET_TAG_ID = 24;

    // Servo tuning
    private static final double SERVO_CENTER = 0.5;
    private static final double DEG_TO_SERVO = 1.0 / 120.0;

    private Webcam aprilTagWebcam;
    private Servo cameraServo;

    @Override
    public void init() {
        aprilTagWebcam = new Webcam();
        aprilTagWebcam.init(hardwareMap, telemetry);

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(SERVO_CENTER);

    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

        if (tag != null) {
            double bearingDeg = tag.ftcPose.bearing;

            double servoPos = SERVO_CENTER - (bearingDeg * DEG_TO_SERVO);

            servoPos = clamp(servoPos, 0.0, 1.0);
            cameraServo.setPosition(servoPos);

            telemetry.addLine("Tag FOUND");
            telemetry.addData("Bearing (deg)", "%.2f", bearingDeg);
            telemetry.addData("Servo Pos", "%.3f", servoPos);
        }
        else{
            telemetry.addLine("Tag NOT FOUND");
        }

        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
