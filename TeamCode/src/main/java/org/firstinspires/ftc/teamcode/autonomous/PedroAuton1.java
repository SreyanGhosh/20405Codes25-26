package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroAuton1")
@Configurable
public class PedroAuton1 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private DcMotorEx rightLauncherMotor, leftLauncherMotor;
    private DcMotor intakeMotor;
    private Servo transferServo;
    private ElapsedTime intakeTimer = new ElapsedTime(); // New timer for intake
    private int ballsLaunched = 0; // Add this variable to count launched balls.


    // ------------------------
    //   LAUNCHER CONSTANTS
    // ------------------------
    private static final double kP = 5.1;
    private static final double kI = 0.0;
    private static final double kD = 0.5;
    private static final double kF = 19.2;
    private static final double TICKS_PER_REV = 28;
    private static final double RPM_TO_TPS = TICKS_PER_REV / 60.0;
    private static final double AUTON_RPM = 1359;
    private static final double AUTON_TARGET_VELOCITY = AUTON_RPM * RPM_TO_TPS;

    private double getLauncherVelocity() {
        return (rightLauncherMotor.getVelocity() + leftLauncherMotor.getVelocity()) / 2;
    }

    @Override
    public void init() {
        rightLauncherMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");
        leftLauncherMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferServo = hardwareMap.get(Servo.class, "transfer");

        leftLauncherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        leftLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        transferServo.setPosition(0.67);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 135, Math.toRadians(180)));
        paths = new Paths(follower);

        pathState = 0;
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        panelsTelemetry.debug("Status", "Started autonomous");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                pathState++;
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;
            case 2:
                // Spin up the launcher motors
                rightLauncherMotor.setVelocity(AUTON_TARGET_VELOCITY);
                leftLauncherMotor.setVelocity(AUTON_TARGET_VELOCITY);

                panelsTelemetry.debug("Launcher Velocity", getLauncherVelocity());
                if (getLauncherVelocity() >= AUTON_TARGET_VELOCITY * 0.99) {
                    // If the launcher is at speed, start the launching process.
                    intakeTimer.reset();
                    intakeMotor.setPower(0.75); // Start the intake
                    pathState++;
                }
                break;
            case 3:
                // Launch one ball and wait for the intake to run for 100ms.
                if (intakeTimer.milliseconds() >= 100) {
                    intakeMotor.setPower(0); // Stop the intake
                    ballsLaunched++; // Increment the counter

                    if (ballsLaunched < 2) {
                        intakeTimer.reset(); // Reset timer for the next ball
                        pathState = 2; // Go back to the launch state for the next ball.
                    } else {
                        pathState++; // All balls launched, move to the end state.
                    }
                }
                break;
            case 4:
                // Autonomous is complete.
                panelsTelemetry.debug("Status", "Autonomous complete");
                rightLauncherMotor.setVelocity(0); // Stop the launchers.
                leftLauncherMotor.setVelocity(0);
                pathState++; // Transition to the final, inactive state.
                break;
            case 5:
                pathState++;
                break;
            case 6:
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;
            case 7:
                break;
            default:
                // Remain in a final, inactive state.
                break;
        }
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88, 135), new Pose(148, 135))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(138, 135), new Pose(167, 167))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}
