package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {

    private Servo hoodServo;

    private double hoodPos = 0.5;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double k = 0.01; // linear factor

    private boolean hasSeenDistance = false;

    private static final double m = -0.015;
    private static final double b = 0.65;



    public void init(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(hoodPos);
    }

    public void update(double distance, boolean hasTag) {
        if (hasTag) {
            // Compute hood position based on distance
            hoodPos = clamp(k * distance, SERVO_MIN, SERVO_MAX);
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
