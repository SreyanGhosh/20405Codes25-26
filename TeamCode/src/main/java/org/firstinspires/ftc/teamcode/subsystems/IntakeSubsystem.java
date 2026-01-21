package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private final DcMotor intake;

    public IntakeSubsystem(HardwareMap hw) {
        intake = hw.dcMotor.get("intakeMotor");
    }

    public void run(boolean enabled) {
        intake.setPower(enabled ? -1 : 0);
    }

    public void runIntake() {
        intake.setPower(1);
    }
    public void stop() {
        intake.setPower(0);
    }
}
