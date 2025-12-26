package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "FlywheelVelocityTuner")
public class FlywheelVelocityTuner extends OpMode {

    Webcam webcam = new Webcam();
    private DcMotorEx flywheel;
    private ElapsedTime timer = new ElapsedTime();
    private double targetVelocity = 2000;
    private Servo hoodServo;
    private double hoodPos = 0;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    // UPDATE THESE VALUES

    private static final double kA = -0.015;
    private static final double kB = 0.65;
    private static final double kC = 0;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        webcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        updateTargetVelocity();
        webcam.update();
        AprilTagDetection tag = webcam.getTagBySpecificId(24);

        hoodServo.setPosition(hoodPos);

        if (tag != null) {
            double distance = tag.ftcPose.range;
            hoodPos = clamp((kA * distance * distance) + kB * distance + kC, SERVO_MIN, SERVO_MAX);
            telemetry.addData("Distance", distance);
            if (tag.ftcPose.bearing > 4) {
                telemetry.addLine("Align Robot");
            } else {
                telemetry.addLine("Aligned");
            }
        } else {
            telemetry.addLine("No Tag Found");
        }

        if (gamepad1.right_trigger > 0.05) {
            runFor(targetVelocity, 6);
        } else if (gamepad1.left_trigger > 0.05) {
            flywheel.setVelocity(0);
        }

        telemetry.addLine("Tune from 5 different distances, then input values into demos");
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", flywheel.getVelocity());
        telemetry.update();
    }

    public void runFor(double velocity, double time) {
        timer.reset();
        if (timer.seconds() < time) {
            flywheel.setVelocity(velocity);
        }
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public void updateTargetVelocity() {
        if (gamepad1.dpad_up) {
            targetVelocity += 20;
        } else if (gamepad1.dpad_down) {
            targetVelocity -= 20;
        }


    }
}
