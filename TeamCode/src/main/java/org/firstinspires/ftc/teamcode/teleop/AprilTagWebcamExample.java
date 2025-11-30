package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTagWebcamExample")
public class AprilTagWebcamExample extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // update the vision portal
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        if (id20 != null) {
            telemetry.addLine("Tag FOUND");
            telemetry.addData("id20 String", id20.toString());

        } else {
            telemetry.addLine("Tag NOT FOUND");
        }

        telemetry.update();


    }
}
