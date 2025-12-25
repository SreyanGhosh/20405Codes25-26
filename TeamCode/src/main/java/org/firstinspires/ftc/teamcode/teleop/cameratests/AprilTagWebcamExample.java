package org.firstinspires.ftc.teamcode.teleop.cameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTagWebcamExample")
public class AprilTagWebcamExample extends OpMode {


    Webcam aprilTagWebcam = new Webcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {


        // updateFlywheel the vision portal
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
        double distance = id24.ftcPose.range;

        if (id24 != null) {
            telemetry.addLine("Tag FOUND");
            aprilTagWebcam.displayDetectionTelemetry(id24);

        } else {
            telemetry.addLine("Tag NOT FOUND");
        }

        telemetry.update();


    }
}
