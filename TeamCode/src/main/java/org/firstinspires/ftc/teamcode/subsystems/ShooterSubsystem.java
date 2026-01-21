package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterSubsystem {

    private final DcMotorEx flywheel;

    private boolean enabled = false;

    public ShooterSubsystem(DcMotorEx flywheel) {
        this.flywheel = flywheel;
    }

    /** Call every loop */
    public void update(boolean togglePressed, double targetVelocity) {
        if (togglePressed) {
            enabled = !enabled;
        }
        flywheel.setVelocity(enabled ? targetVelocity : 0);
    }

    public boolean isEnabled() {
        return enabled;
    }
}
