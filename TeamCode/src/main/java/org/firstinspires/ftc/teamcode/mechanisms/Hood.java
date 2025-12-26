package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {

    private Servo hoodServo;

    private double hoodPos = 0.5;
    private boolean hasSeenDistance = false;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    // UPDATE THESE VALUES

    private static final double kA = -0.015;
    private static final double kB = 0.65;
    private static final double kC = 0;



    public void init(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(hoodPos);
    }

    public void update(double distance, boolean hasTag) {
        if (hasTag) {
            // Compute hood position based on distance
            hoodPos = clamp((kA * distance * distance) + kB * distance + kC, SERVO_MIN, SERVO_MAX);
            hasSeenDistance = true;
        }
        // Else: keep hoodPos the same (hold last position)

        hoodServo.setPosition(hoodPos);
    }

    public double getPosition() {
        return hoodPos;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
