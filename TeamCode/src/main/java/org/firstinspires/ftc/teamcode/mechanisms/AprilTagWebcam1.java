package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * A complete AprilTag webcam wrapper that provides:
 * - Initialization
 * - Tag detection
 * - Telemetry display
 * - Field pose calculation using known tag locations
 */
public class AprilTagWebcam1 {

    // Camera offset relative to robot center (in robot coordinates)
    double camX = 7.5;   // inches forward
    double camY = 0.0;   // inches left
    double camHeadingOffset = 0;  // deg


    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    /** Represents the known pose of a tag on the field */
    public static class FieldTagPose {
        public double x;       // inches
        public double y;       // inches
        public double heading; // degrees

        public FieldTagPose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // You MUST fill in your own FTC field tag locations.
    // *** Replace these values with the real AprilTag coordinates ***
    public FieldTagPose getFieldTagPose(int id) {
        switch (id) {
            case 20: return new FieldTagPose(60, 36, 90);
            case 24: return new FieldTagPose(60, -36, 90);
            default: return null;
        }
    }

    /** Initialize camera + pipeline */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    /** Call each loop to refresh tag data */
    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getTagById(int id) {
        for (AprilTagDetection t : detectedTags) {
            if (t.id == id) return t;
        }
        return null;
    }

    /** Display all detections to telemetry */
    public void displayAllTelemetry() {
        if (detectedTags.isEmpty()) {
            telemetry.addLine("No AprilTags detected.");
            return;
        }

        for (AprilTagDetection tag : detectedTags) {
            displayTagTelemetry(tag);
        }
    }

    /** Display info for one tag */
    public void displayTagTelemetry(AprilTagDetection tag) {
        telemetry.addLine("====== TAG ID " + tag.id + " ======");

        if (tag.metadata != null) {
            telemetry.addLine("Name: " + tag.metadata.name);
            telemetry.addData("X/Y/Z (in)", "%.1f, %.1f, %.1f",
                    tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
            telemetry.addData("Yaw/Pitch/Roll (deg)", "%.1f, %.1f, %.1f",
                    tag.ftcPose.yaw, tag.ftcPose.pitch, tag.ftcPose.roll);
            telemetry.addData("Range (in)", "%.1f", tag.ftcPose.range);
            telemetry.addData("Bearing (deg)", "%.1f", tag.ftcPose.bearing);

            // Also output field pose if known
            FieldTagPose known = getFieldTagPose(tag.id);
            if (known != null) {
                double[] pos = computeRobotFieldPose(tag, known);
                telemetry.addData("Robot Field X/Y/Heading",
                        "%.1f, %.1f, %.1f", pos[0], pos[1], pos[2]);
            }

        } else {
            telemetry.addLine("No Tag Metadata!");
        }
        telemetry.addLine();
    }

    /**
     * Computes robot field pose based on tag detection and known tag location.
     * Returns array: [robotX, robotY, robotHeading] in field space.
     */
    public double[] computeRobotFieldPose(AprilTagDetection det, FieldTagPose tagPose) {

        double rx = det.ftcPose.x;   // camera X wrt tag
        double ry = det.ftcPose.y;   // camera Y wrt tag
        double rYaw = det.ftcPose.yaw;

        double thTag = Math.toRadians(tagPose.heading);

        // First: Convert camera pose into field frame
        double field_rx = rx * Math.cos(thTag) - ry * Math.sin(thTag);
        double field_ry = rx * Math.sin(thTag) + ry * Math.cos(thTag);

        // Add tag origin offset
        double camFieldX = tagPose.x + field_rx;
        double camFieldY = tagPose.y + field_ry;

        // Now compute robot heading (camera heading included)
        double robotHeading =
                AngleUnit.normalizeDegrees(tagPose.heading + rYaw - camHeadingOffset);

        double thRobot = Math.toRadians(robotHeading);

        // Rotate camera offset into field frame
        double robotOffsetX = camX * Math.cos(thRobot) - camY * Math.sin(thRobot);
        double robotOffsetY = camX * Math.sin(thRobot) + camY * Math.cos(thRobot);

        // Robot center = camera position + camera → robot offset
        double robotX = camFieldX - robotOffsetX;
        double robotY = camFieldY - robotOffsetY;

        return new double[]{robotX, robotY, robotHeading};
    }


    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
