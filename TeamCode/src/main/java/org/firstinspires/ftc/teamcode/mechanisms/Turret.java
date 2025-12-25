package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class Turret {

    private Servo cameraServo;

    private double servoPos = 0.5;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double kP = 0.005;
    private static final double MAX_STEP = 0.002;
    private static final double DEADBAND_RAD = Math.toRadians(1.5);

    public void init(HardwareMap hardwareMap) {
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(servoPos);
    }

    public void update(double yawErrorRad) {

        if (Math.abs(yawErrorRad) > DEADBAND_RAD) {
            double desiredDelta = yawErrorRad * kP;
            double clampedDelta = clamp(desiredDelta, -MAX_STEP, MAX_STEP);
            servoPos += clampedDelta;
        }

        servoPos = clamp(servoPos, SERVO_MIN, SERVO_MAX);
        cameraServo.setPosition(servoPos);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
