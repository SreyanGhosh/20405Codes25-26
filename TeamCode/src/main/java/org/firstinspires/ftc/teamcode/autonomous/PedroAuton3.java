package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PedroAuton3 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {GO_LAUNCH, SHOOT, ALIGNED_FIRST, INTAKE_ONE, ALIGNED_SECOND, INTAKE_TWO, ALIGNED_THIRD,INTAKE_THREE, DONE, WAIT_LAUNCH}
    private PathState pathState;

    private final Pose startPose = new Pose(81.2, 134.56, Math.toRadians(180));
    private final Pose launchPose = new Pose(81.2, 78, Math.toRadians(45));
    private final Pose alignPoseOne = new Pose(102.36, 83.73, Math.toRadians(0));
    private final Pose intakePoseOne = new Pose(128.12, 83.73, Math.toRadians(0));
    private final Pose alignPoseTwo = new Pose(102.36, 59.81, Math.toRadians(0));
    private final Pose intakePoseTwo = new Pose(128.12,59.81, Math.toRadians(0));
    private final Pose alignPoseThree = new Pose(102.36, 35.5, Math.toRadians(0));
    private final Pose intakePoseThree = new Pose(128.12, 35.5, Math.toRadians(0));

    private PathChain goLaunch, alignFirst, intakeFirst, alignSecond, intakeSecond, alignThird, intakeThird;
    double roundsShot = 0;
    public void buildPaths() {

        Pose currentPose = follower.getPose();

        goLaunch = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, launchPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), launchPose.getHeading())
                .build();

        alignFirst = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, alignPoseOne))
                .setLinearHeadingInterpolation(launchPose.getHeading(), alignPoseOne.getHeading())
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(alignPoseOne, intakePoseOne))
                .setLinearHeadingInterpolation(alignPoseOne.getHeading(), intakePoseOne.getHeading())
                .build();
        alignSecond = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, alignPoseTwo))
                .setLinearHeadingInterpolation(launchPose.getHeading(), alignPoseTwo.getHeading())
                .build();
        intakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(alignPoseTwo, intakePoseTwo))
                .setLinearHeadingInterpolation(alignPoseTwo.getHeading(), intakePoseTwo.getHeading())
                .build();
        alignThird = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, alignPoseThree))
                .setLinearHeadingInterpolation(launchPose.getHeading(), alignPoseThree.getHeading())
                .build();
        intakeThird = follower.pathBuilder()
                .addPath(new BezierLine(alignPoseThree, intakePoseThree))
                .setLinearHeadingInterpolation(alignPoseThree.getHeading(), intakePoseThree.getHeading())
                .build();
    }

    // State machine handler
    public void statePathUpdate() {

        switch (pathState) {

            case GO_LAUNCH:
                follower.followPath(goLaunch, true);   // start the path ONCE
                setPathState(PathState.WAIT_LAUNCH);   // immediately go to waiting state
                break;

            case WAIT_LAUNCH:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT);     // now safe to shoot
                }
                break;

            case SHOOT:
                if (!follower.isBusy()) {

                    // increment number of shots
                    roundsShot++;

                    telemetry.addData("Shots Fired", roundsShot);

                    if (roundsShot == 1) {
                        setPathState(PathState.ALIGNED_FIRST);

                    } else if (roundsShot == 2) {
                        setPathState(PathState.ALIGNED_SECOND);

                    } else if (roundsShot == 3) {
                        setPathState(PathState.ALIGNED_THIRD);

                    } else if (roundsShot == 4){
                        setPathState(PathState.DONE);
                        telemetry.addLine("Finished all cycles!");
                    }
                }
                break;

            case ALIGNED_FIRST:
                follower.followPath(alignFirst, true);
                setPathState(PathState.INTAKE_ONE);
                break;
            case INTAKE_ONE:
                follower.followPath(intakeFirst, true);
                setPathState(PathState.GO_LAUNCH);
                break;
            case ALIGNED_SECOND:
                follower.followPath(alignSecond, true);
                setPathState(PathState.INTAKE_TWO);
                break;
            case INTAKE_TWO:
                follower.followPath(intakeSecond, true);
                setPathState(PathState.GO_LAUNCH);
                break;
            case ALIGNED_THIRD:
                follower.followPath(alignThird, true);
                setPathState(PathState.INTAKE_THREE);
                break;
            case INTAKE_THREE:
                follower.followPath(intakeThird, true);
                setPathState(PathState.GO_LAUNCH);
                break;
            case DONE:
                follower.breakFollowing();
                telemetry.addLine("Autonomous complete.");
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

        follower = Constants.createFollower(hardwareMap);

        pathTimer = new Timer();
        opModeTimer = new Timer();

        // Set the robot pose FIRST
        follower.setPose(startPose);

        // Now paths can be built safely
        buildPaths();

        // Begin at this state
        pathState = PathState.GO_LAUNCH;
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

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("State Time", pathTimer.getElapsedTimeSeconds());
    }
}