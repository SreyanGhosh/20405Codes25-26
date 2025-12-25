package org.firstinspires.ftc.teamcode.teleop.fullcodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class FullRobotCode6 extends OpMode {

    Drivetrain drivetrain = new Drivetrain();
    Flywheel flywheel = new Flywheel();
    Intake intake = new Intake();
    Kicker kicker = new Kicker();
    Turret turret = new Turret();
    Hood hood = new Hood();
    Webcam webcam = new Webcam();

    private static final int TARGET_ID = 24;

    private static final double VELOCITY_MULTIPLIER = 1000;
    private static final double SPEED_TOLERANCE = 50;

    private boolean shootRequested = false;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        flywheel.init(hardwareMap);
        intake.init(hardwareMap);
        kicker.init(hardwareMap);
        turret.init(hardwareMap);
        hood.init(hardwareMap);
        webcam.init(hardwareMap, telemetry);
    }
    @Override
    public void loop() {

        // ===== Normal robot control =====
        drivetrain.update(gamepad1);
        intake.update(gamepad2);

        // ===== Vision =====
        webcam.update();
        AprilTagDetection tag = webcam.getTagBySpecificId(TARGET_ID);

        boolean hasTag = (tag != null);
        double distance = hasTag ? tag.ftcPose.range : 0;

        // Flywheel updates internally
        flywheel.updateFromDistance(distance, hasTag);

        // Request shooting
        if (gamepad2.a) shootRequested = true;

        // Fire only when ready
        if (shootRequested && !kicker.isBusy()
                && flywheel.atSpeed(flywheel.getTargetVelocity(), SPEED_TOLERANCE)) {

            kicker.startKicking();
            shootRequested = false;
        }

        hood.update(distance, hasTag);


        // Kicker update
        kicker.update();

        // ===== Telemetry =====
        telemetry.addData("Target Velocity", flywheel.getTargetVelocity());
        telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
        telemetry.addData("Kicker Busy", kicker.isBusy());
        telemetry.update();
    }

}