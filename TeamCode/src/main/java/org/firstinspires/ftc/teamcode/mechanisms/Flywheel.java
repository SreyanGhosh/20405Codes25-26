package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {

    private DcMotorEx flywheel;

    private static final double kP = 0.01;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0002;


    private static final double kA = 1000;
    private static final double kB = 1000;
    private static final double kH = 1000;
    private static final double kK = 1000;
    private static final double DEFAULT_VELOCITY = 1400;

    private double targetVelocity = DEFAULT_VELOCITY;
    private boolean hasSeenTag = false;
    private ElapsedTime timer = new ElapsedTime();


    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF)
        );
    }

    public void setVelocity(double velocity) {
        flywheel.setVelocity(velocity);
    }

    public void stop() {
        setVelocity(0);
    }

    public void updateFromDistance(double distance, boolean hasTag) {

        if (hasTag) {
            targetVelocity = kA * Math.sqrt(kB * (distance - kH)) + kK;
        }
        else if (!hasSeenTag) {
            targetVelocity = DEFAULT_VELOCITY;
        }

        if (hasTag) hasSeenTag = true;

        flywheel.setVelocity(targetVelocity);
    }

    public void runFor(double velocity, double time) {
        timer.reset();
        if (timer.seconds() < time) {
            flywheel.setVelocity(velocity);
        }
    }


    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean atSpeed(double target, double tolerance) {
        return Math.abs(getVelocity() - target) <= tolerance;
    }



}
