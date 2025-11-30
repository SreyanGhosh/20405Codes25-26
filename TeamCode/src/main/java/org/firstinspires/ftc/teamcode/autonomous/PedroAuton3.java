package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PedroAuton3 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {GO_LAUNCH, SHOOT_PRELOAD, INTAKE_ONE, INTAKE_TWO, INTAKE_THREE}
    PathState pathState;

    private final Pose startPose = new Pose(81.20127795527158, 134.56869009584665, Math.toRadians(180));
    private final Pose launchPose = new Pose(81.20127795527158, 78, Math.toRadians(45));
    private final Pose intakeOne = new Pose(102.36421725239616, 83.73162939297124, Math.toRadians(0));

    private PathChain goLaunch, intakeFirst;

    public void buildPaths() {
        goLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intakeOne))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intakeOne.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case GO_LAUNCH:
                follower.followPath(goLaunch, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    // Flywheel logic
                    telemetry.addLine("Path Done");
                }
                setPathState(PathState.INTAKE_ONE);
                break;
            case INTAKE_ONE:


            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.GO_LAUNCH;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Other init mechanisms

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());

    }
}