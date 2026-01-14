package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTag Servo Turret")
public class AprilTagCameraLockSmoothed extends OpMode {

    // ================= HARDWARE =================
    private Servo turret;
    private Webcam webcam;

    // ================= CONFIG =================
    private static final int TAG_ID = 22;

    // Servo limits
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    // Control tuning
    private static final double kP = 0.8;
    private static final double MAX_SERVO_STEP = 0.008; // limits angular speed
    private static final double ANGLE_DEADBAND = Math.toRadians(3.0); // LARGE deadband

    // Filtering
    private static final double TARGET_FILTER_ALPHA = 0.90; // higher = smoother

    // ================= STATE =================
    private double turretAngleRad = 0.0;        // current turret angle (-π, π]
    private double targetAngleFiltered = 0.0;   // filtered target angle
    private double servoPos = 0.5;

    @Override
    public void init() {

        turret = hardwareMap.get(Servo.class, "turret");

        // 🔴 REVERSE THIS IF TURRET MOVES THE  WAY
        // turret.setDirection(Servo.Direction.REVERSE);
        turret.setDirection(Servo.Direction.REVERSE);
        webcam = new Webcam();
        webcam.init(hardwareMap, telemetry);

        // Start facing forward
        turretAngleRad = 0.0;
        targetAngleFiltered = 0.0;
        servoPos = angleToServo(turretAngleRad);
        turret.setPosition(servoPos);

        telemetry.addLine("Stable AprilTag Servo Turret Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        webcam.update();
        AprilTagDetection tag = webcam.getTagBySpecificId(TAG_ID);

        if (tag != null && tag.metadata != null) {

            // ================= TARGET ANGLE (UNIT CIRCLE) =================
            double rawTargetAngle = Math.atan2(
                    tag.ftcPose.x,   // left/right (flip sign here if needed)
                    tag.ftcPose.z    // forward
            );

            // ================= FILTER TARGET =================
            targetAngleFiltered = angleWrap(
                    TARGET_FILTER_ALPHA * targetAngleFiltered
                            + (1.0 - TARGET_FILTER_ALPHA) * rawTargetAngle
            );

            // ================= ANGLE ERROR =================
            double error = angleWrap(targetAngleFiltered - turretAngleRad);

            // ================= LARGE DEADBAND =================
            if (Math.abs(error) > ANGLE_DEADBAND) {

                double delta = clamp(
                        kP * error,
                        -MAX_SERVO_STEP,
                        MAX_SERVO_STEP
                );

                turretAngleRad = angleWrap(turretAngleRad + delta);
                servoPos = angleToServo(turretAngleRad);
                turret.setPosition(servoPos);

                telemetry.addLine("Tracking Tag");
                telemetry.addData("Raw Target (deg)", Math.toDegrees(rawTargetAngle));
                telemetry.addData("Filtered Target (deg)", Math.toDegrees(targetAngleFiltered));
                telemetry.addData("Turret Angle (deg)", Math.toDegrees(turretAngleRad));
                telemetry.addData("Error (deg)", Math.toDegrees(error));


            } else {
                telemetry.addLine("Error less than DEADBAND — Holding Position");
            }

        } else {
            telemetry.addLine("No Tag Detected — Holding Position");
        }

        telemetry.addData("Servo Position", "%.3f", servoPos);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPosition(servoPos);
        webcam.stop();
    }

    // ================= MATH UTILITIES =================

    /** Wrap angle to (-π, π] */
    private double angleWrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /** Map angle (-π, π] → servo [0, 1] */
    private double angleToServo(double angleRad) {
        double normalized = (angleRad + Math.PI) / (2.0 * Math.PI);
        return clamp(normalized, SERVO_MIN, SERVO_MAX);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}