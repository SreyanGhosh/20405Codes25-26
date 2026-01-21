package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class FollowerSubsystem {

    private final Follower follower;

    public FollowerSubsystem(HardwareMap hw) {
        follower = Constants.createFollower(hw);
        follower.setPose(new Pose(135, 9, Math.toRadians(90)));
    }

    public void update() {
        follower.update();
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public double getDistanceTo(double targetX, double targetY) {
        Pose robotPose = getPose(); // use follower's current pose
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();
        return Math.hypot(dx, dy);
    }


}
