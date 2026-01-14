package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Turret {

    // ==== CONFIG ====
    private static final int TARGET_TAG_ID = 22;
    private static final double FRAME_WIDTH = 640.0;

    // Control tuning
    private static final double kP = 1.20;
    private static final double kD = 0.00;
    private static final double AIM_TOLERANCE = 0.03;


    // CR Servo limits
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = 1.0;

    // ==== HARDWARE ====
    private final CRServo turret;

    // ==== STATE ====
    private final Webcam webcam;

    private double lastError = 0;
    private double aimError = 0;
    private boolean tagDetected = false;
    private double distanceFromTag = 0;

    public Turret(HardwareMap hardwareMap, Webcam webcam) {
        this.webcam = webcam;
        turret = hardwareMap.get(CRServo.class, "turret");
        turret.setPower(0);
    }

    // Same role as their periodic()
    public void update() {
        AprilTagDetection tag = webcam.getTagBySpecificId(TARGET_TAG_ID);

        if (tag == null) {
            tagDetected = false;
            return;
        }

        tagDetected = true;
        distanceFromTag = tag.ftcPose.range;

        aimError = (tag.center.x - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2);
    }

    // Same role as their autoAim()
    public boolean autoAim() {
        if (!tagDetected) {
            turret.setPower(0);
            return false;
        }

        if (Math.abs(aimError) < AIM_TOLERANCE) {
            turret.setPower(0);
            return true;
        }

        double derivative = aimError - lastError;
        double output = kP * aimError + kD * derivative;

        output = clamp(output, -MAX_POWER, MAX_POWER);

        if (Math.abs(output) < MIN_POWER) {
            output = Math.signum(output) * MIN_POWER;
        }

        turret.setPower(-output);

        lastError = aimError;
        return false;
    }

    public double getDistance() {
        return distanceFromTag;
    }

    public boolean isAimed() {
        return Math.abs(aimError) < AIM_TOLERANCE;
    }

    public boolean hasTarget() {
        return tagDetected;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
