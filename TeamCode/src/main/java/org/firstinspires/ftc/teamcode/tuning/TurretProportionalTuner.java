
package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "TurretProportionalTuner")
public class TurretProportionalTuner extends OpMode {

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

    private double kP = 0.0067;

    private double MAX_STEP = 0.0015;
    private double kD = 0.015;          // Damping
    private double filteredYaw = 0.0;
    private double filterAlpha = 0.12;   // 0–1 (higher = more responsive)
    private double lastError = 0.0;


    private static final double DEADBAND_RAD = Math.toRadians(2);

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
            kP += 0.0005;
        } else if (gamepad1.dpad_down) {
            kP -= 0.0005;
        }

        if (gamepad1.y) {
            MAX_STEP += 0.0005;
        } else if (gamepad1.a) {
            MAX_STEP -= 0.0005;
        }

        if(gamepad1.right_bumper) {
            kD += 0.003;
        } else if (gamepad1.left_bumper) {
            kD -= 0.003;
        }



        if (tag != null) {

            double x = tag.ftcPose.x;
            double z = tag.ftcPose.z;
            double rawYaw = Math.atan2(x, z);

// ----- Low-pass filter vision noise -----
            filteredYaw = filteredYaw * (1 - filterAlpha) + rawYaw * filterAlpha;

            double error = filteredYaw;
            double derivative = error - lastError;

// ----- PD Control -----
            double correction = kP * error + kD * derivative;

// Step limiting for safety
            correction = clamp(correction, -MAX_STEP, MAX_STEP);

            if (Math.abs(error) > DEADBAND_RAD) {
                servoPos -= correction;
            }

            lastError = error;


            servoPos = clamp(servoPos, SERVO_MIN, SERVO_MAX);
            cameraServo.setPosition(servoPos);

            telemetry.addLine("STATUS: Tag Locked");
            telemetry.addData("Yaw Error (deg)", Math.toDegrees(rawYaw));
            telemetry.addData("Servo Pos", "%.3f", servoPos);

        } else {
            // Tag lost: Maintain last position smoothly
            cameraServo.setPosition(servoPos);
            telemetry.addLine("STATUS: Tag Lost - Holding last position");
        }

        telemetry.addData("Proportional", kP);
        telemetry.addData("Max Step", MAX_STEP);
        telemetry.addData("kD", kD);
        telemetry.update();
    }
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}