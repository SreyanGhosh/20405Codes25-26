package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class FlywheelPIDFTuner extends OpMode {

    private double velocity = 2000;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    private DcMotorEx flywheel;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );
    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger > 0.05) {
            flywheel.setVelocity(velocity);
        } else if (gamepad1.left_trigger > 0.05) {
            flywheel.setVelocity(0);
        }

        if (gamepad1.dpad_up) {
            kP += 0.05;
        } if (gamepad1.dpad_right) {
            kI += 0.05;
        } if (gamepad1.dpad_down) {
            kD += 0.05;
        } if (gamepad1.dpad_left) {
            kF += 0.05;
        } else if (gamepad1.y) {
            kP -= 0.05;
        } else if (gamepad1.b) {
            kI -= 0.05;
        } else if (gamepad1.a) {
            kD -= 0.05;
        } else if (gamepad1.x) {
            kF -= 0.05;
        }


        telemetry.addLine("Make sure camera distance is 30 inches");

        telemetry.addData("Proportional: DPAD Up (+), Y (-) ", kP);
        telemetry.addData("Integral: DPAD Right (+), B (-)", kI);
        telemetry.addData("Derivative: DPAD Down (+), A (-)", kD);
        telemetry.addData("Feedforward: DPAD Left (+), X (-)", kF);

        telemetry.addData("Target Velocity", velocity);
        telemetry.addData("Current Velocity", flywheel.getVelocity());
        telemetry.update();

    }
}
