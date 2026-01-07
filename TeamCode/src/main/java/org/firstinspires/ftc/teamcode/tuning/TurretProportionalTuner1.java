package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "TurretProportionalTuner1")
public class TurretProportionalTuner1 extends OpMode {

    // Vision
    private Webcam aprilTagWebcam = new Webcam();

    // CR Servo
    private CRServo cameraServo;

    // ===== TUNING CONSTANTS =====
    private static final int TARGET_TAG_ID = 24;
    private static final double DEADBAND_RAD = Math.toRadians(1.5);

    private double kP = 0.35;
    private double kD = 0.06;

    private double filterAlpha = 0.12;

    // Internal control state
    private double filteredYaw = 0.0;
    private double lastError = 0.0;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        cameraServo = hardwareMap.get(CRServo.class, "cameraServo");
        cameraServo.setPower(0);

        telemetry.addLine("CR Turret Tracker Ready");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

        // Live tuning
        if (gamepad1.dpad_up)   kP += 0.01;
        if (gamepad1.dpad_down) kP -= 0.01;

        if (gamepad1.y) kD += 0.002;
        if (gamepad1.a) kD -= 0.002;

        if (tag != null) {
            double x = tag.ftcPose.x;
            double z = tag.ftcPose.z;

// Raw yaw from vision
            double rawYaw = Math.atan2(x, z);

// ----- Heavy filtering -----
            filteredYaw = filteredYaw * 0.85 + rawYaw * 0.15;

            double error = filteredYaw;

// Derivative with filtering to avoid noise
            double derivative = 0.7 * (error - lastError);

// Nonlinear scaling: slow near center, faster when far away
            double scaledError = Math.signum(error) * Math.sqrt(Math.abs(error));

// PD control → velocity
            double power = kP * scaledError + kD * derivative;

// Very tight speed limit
            power = clamp(power, -0.35, 0.35);

// Soft deadband
            if (Math.abs(error) < Math.toRadians(1.8)) {
                power = 0;
            }

// Apply power
            cameraServo.setPower(power);
            lastError = error;


            cameraServo.setPower(power);
            lastError = error;

            telemetry.addLine("STATUS: Tag Locked");
            telemetry.addData("Yaw Error (deg)", Math.toDegrees(error));
            telemetry.addData("Power", "%.3f", power);

        } else {
            cameraServo.setPower(0);
            telemetry.addLine("STATUS: Tag Lost");
        }

        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
        cameraServo.setPower(0);
    }
}