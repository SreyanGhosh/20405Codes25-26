package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Flywheel PID Velocity Tuner")
public class FlywheelVelocityTuner extends OpMode {

    private DcMotorEx flywheel;
    private Webcam webcam = new Webcam();

    private double targetVelocity = 2000;

    // PIDF values (START SAFE

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(0.00215, 0, 0.0001, 19.9)
        );

        webcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        // --- Flywheel enable ---
        if (gamepad1.right_bumper) {
            flywheel.setVelocity(targetVelocity);
        }
        if (gamepad1.left_bumper) {
            flywheel.setVelocity(0);
        }
        // --- AprilTag ---
        webcam.update();
        AprilTagDetection tag = webcam.getTagBySpecificId(24);
        if (tag != null) {
            telemetry.addData("Distance (in)", tag.ftcPose.range);
        }

        // --- Telemetry ---
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Actual Vel", flywheel.getVelocity());
        telemetry.update();
    }
}

