package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTag Camera Center Lock")
public class AprilTagCameraCenter extends OpMode {

    // Vision
    private Webcam aprilTagWebcam = new Webcam();

    // Servo
    private Servo cameraServo;

    // Servo position
    private double servoPos = 0.5;

    // ===== TUNING CONSTANTS =====
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    // Gain: servo change per radian
    private double kP = 0.03;

    // Ignore tiny errors
    private static final double DEADBAND_RAD = Math.toRadians(1);

    // Tag to track
    private static final int TARGET_TAG_ID = 22;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(servoPos);

        telemetry.addLine("AprilTag Camera Center Lock Initialized");
    }

    @Override
    public void loop() {

        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

        if (tag != null) {

            // Camera-relative position of tag center
            double x = tag.ftcPose.x;   // left/right (inches)
            double z = tag.ftcPose.z;   // forward (inches)

            // Horizontal angle to tag center (radians)
            double yawError = Math.atan2(x, z);

            // Apply P control
            if (Math.abs(yawError) > DEADBAND_RAD) {
                servoPos -= yawError * kP;
            }

            // Clamp servo
            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
            cameraServo.setPosition(servoPos);

            telemetry.addLine("Tag FOUND");
            telemetry.addData("Yaw Error (deg)", Math.toDegrees(yawError));
            telemetry.addData("Servo Position", servoPos);

        } else {
            telemetry.addLine("Tag NOT FOUND");
        }

        telemetry.update();
    }



    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
