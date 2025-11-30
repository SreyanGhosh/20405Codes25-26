package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PedroAuton2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {GO_LAUNCH1, SHOOT_PRELOAD}
    PathState pathState;

    private final Pose startPose = new Pose(88, 135, Math.toRadians(180));
    private final Pose launchPose = new Pose(148,135, Math.toRadians(45));

    private PathChain goLaunch;

    public void buildPaths() {
        goLaunch = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case GO_LAUNCH1:
                follower.followPath(goLaunch, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    // Flywheel logic
                    telemetry.addLine("Path Done");
                }
                break;
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
        pathState = PathState.GO_LAUNCH1;
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
