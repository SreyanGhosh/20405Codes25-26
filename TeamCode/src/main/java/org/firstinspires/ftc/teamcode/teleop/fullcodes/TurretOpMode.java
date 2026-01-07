package org.firstinspires.ftc.teamcode.teleop.fullcodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "TurretOnly")
public class TurretOpMode extends OpMode {

    Turret turret;
    Webcam webcam = new Webcam();

    private static final int TARGET_ID = 24;

    @Override
    public void init() {
        webcam.init(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, webcam);
    }

    @Override
    public void loop() {

        // ===== Vision =====
        webcam.update();  // update the camera once
        AprilTagDetection tag = webcam.getTagBySpecificId(TARGET_ID);

        // ===== Turret =====
        turret.update();    // read the latest tag info
        turret.autoAim();      // move CR servo to aim at tag

        // ===== Telemetry =====
        telemetry.addData("Turret Has Tag", turret.hasTarget());
        telemetry.addData("Turret Aimed", turret.isAimed());
        telemetry.addData("Turret Aim Error", turret.hasTarget() ? turret.getDistance() : 0);
        telemetry.update();
    }
}
