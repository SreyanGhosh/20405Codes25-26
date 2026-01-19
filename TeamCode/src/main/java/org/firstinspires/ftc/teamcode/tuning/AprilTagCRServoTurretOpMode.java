package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Axon AprilTag Turret (Ultra Stable)", group = "Vision")
public class AprilTagCRServoTurretOpMode extends OpMode {

    /* ===================== HARDWARE ===================== */
    private Servo turret;
    private Webcam webcam;
    private IMU imu;

    /* ===================== CONFIG ===================== */

    private static final int TAG_ID = 24;

    // Axon servo ~270°
    private static final double SERVO_RANGE_RAD = Math.toRadians(355);

    // Safe electrical limits
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    // Control tuning (CONSERVATIVE = STABLE)
    private static final double kP = 0.84;
    private static final double MAX_ANGLE_STEP = Math.toRadians(0.65); // rad / loop
    private static final double ANGLE_DEADBAND = Math.toRadians(6.86);

    // Vision smoothing (VERY IMPORTANT)
    private static final double VISION_BLEND = 0.03;

    // Servo command suppression
    private static final double SERVO_EPSILON = 0.003;

    /* ===================== STATE ===================== */

    private double turretAngleRad = 0.0;
    private double filteredTargetRad = 0.0;
    private double lastYawRad = 0.0;
    private double lastServoPos = 0.5;

    /* ===================== INIT ===================== */

    @Override
    public void init() {

        turret = hardwareMap.get(Servo.class, "turret");

        turret.setDirection(Servo.Direction.REVERSE);
        turret.scaleRange(SERVO_MIN, SERVO_MAX);

        webcam = new Webcam();
        webcam.init(hardwareMap, telemetry);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        lastYawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        turretAngleRad = 0.0;
        lastServoPos = angleToServo(turretAngleRad);
        turret.setPosition(lastServoPos);

        telemetry.addLine("Axon Turret Ultra-Stable Initialized");
        telemetry.update();
    }

    /* ===================== LOOP ===================== */

    @Override
    public void loop() {

        /* ---------- IMU integration ---------- */
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double deltaYaw = angleWrap(currentYaw - lastYawRad);
        lastYawRad = currentYaw;

        turretAngleRad = angleWrap(turretAngleRad + deltaYaw);

        /* ---------- Vision ---------- */
        webcam.update();
        AprilTagDetection tag = webcam.getTagBySpecificId(TAG_ID);

        if (tag != null && tag.metadata != null) {

            // Unit-circle correct offset
            double visionAngle = Math.atan2(tag.ftcPose.x, tag.ftcPose.z);

            // Heavy smoothing (complementary filter)
            filteredTargetRad = angleWrap(
                    (1.0 - VISION_BLEND) * filteredTargetRad
                            + VISION_BLEND * visionAngle
            );

            double error = angleWrap(filteredTargetRad - turretAngleRad);

            // LARGE deadband to prevent jitter
            if (Math.abs(error) > ANGLE_DEADBAND) {

                double step = clamp(
                        kP * error,
                        -MAX_ANGLE_STEP,
                        MAX_ANGLE_STEP
                );

                turretAngleRad = angleWrap(turretAngleRad + step);
            }

            telemetry.addLine("Tracking Tag");
            telemetry.addData("Vision (deg)", Math.toDegrees(visionAngle));
            telemetry.addData("Target (deg)", Math.toDegrees(filteredTargetRad));
            telemetry.addData("Error (deg)", Math.toDegrees(error));

        } else {
            telemetry.addLine("No Tag — Holding");
        }

        /* ---------- Servo output (NO MICRO UPDATES) ---------- */
        double servoPos = angleToServo(turretAngleRad);

        if (Math.abs(servoPos - lastServoPos) > SERVO_EPSILON) {
            turret.setPosition(servoPos);
            lastServoPos = servoPos;
        }

        telemetry.addData("Turret Angle (deg)", Math.toDegrees(turretAngleRad));
        telemetry.addData("Servo Pos", "%.3f", lastServoPos);
        telemetry.update();
    }

    @Override
    public void stop() {
        webcam.stop();
    }

    /* ===================== MATH ===================== */

    private double angleToServo(double angleRad) {
        double normalized = (angleRad / SERVO_RANGE_RAD) + 0.5;
        return clamp(normalized, 0.0, 1.0);
    }

    private double angleWrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}