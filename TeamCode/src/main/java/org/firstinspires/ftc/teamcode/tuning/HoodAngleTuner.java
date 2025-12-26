package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "HoodAngleTuner")
public class HoodAngleTuner extends OpMode {

    Flywheel flywheel = new Flywheel();
    Webcam webcam = new Webcam();
    private double hoodPos = 0;
    private Servo hood;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(0);
        webcam.init(hardwareMap, telemetry);
        flywheel.init(hardwareMap);
    }

    @Override
    public void loop() {

        AprilTagDetection tag = webcam.getTagBySpecificId(24);
        if (tag != null) {
            double distance = tag.ftcPose.range;
            telemetry.addData("Distance", distance);
        } else {
            telemetry.addLine("Tag Not Found");
        }

        hood.setPosition(hoodPos);

        if (gamepad1.right_trigger > 0.05) {
            flywheel.runFor(3000, 6);
        } else if (gamepad1.left_trigger > 0.05) {
            flywheel.setVelocity(0);
        }

        if (gamepad1.dpad_up) {
            hoodPos += 0.1;
        } else if (gamepad1.dpad_down) {
            hoodPos -= 0.1;
        }

        telemetry.addLine("Record the distance and the hood angle");
        telemetry.addData("Hood Position", hoodPos);

    }
}
