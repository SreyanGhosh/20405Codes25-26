package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.73944)
            .forwardZeroPowerAcceleration(-37.2888429)
            .lateralZeroPowerAcceleration(-87.6028098)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0.25, 0., 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.25, 0, 0, 0.0,0.01))
            .centripetalScaling(-0.2)
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(83.1830022)
            .yVelocity(64.1120025)
            ;
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("frontRight")
            .strafeEncoder_HardwareMapName("frontLeft")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .forwardPodY(1.5)
            .strafePodX(4.75)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.001982933333)
            .strafeTicksToInches(0.00200985)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
