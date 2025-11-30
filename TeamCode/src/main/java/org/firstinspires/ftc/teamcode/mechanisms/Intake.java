package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intakeMotor;

    private enum IntakeState {
        OFF, INTAKE, REVERSE
    }
    private IntakeState intakeState = IntakeState.OFF;

    private final ElapsedTime intakeTimer = new ElapsedTime();
    double IntakeTimer = intakeTimer.milliseconds();
    public boolean intakeTimerStarted = false;


    public void init(HardwareMap hardwareMap) {

        // Launcher
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Brake behavior
        for (DcMotor m : new DcMotor[]{intakeMotor}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void update() {
        if (gamepad2.a) {
            intakeState = IntakeState.OFF;
        } else if (gamepad2.right_trigger > 0.05) {
            intakeState = IntakeState.INTAKE;
        } else if (gamepad2.left_trigger > 0.05) {
            intakeState = IntakeState.REVERSE;
        }
        switch (intakeState) {
            case OFF:
                intakeMotor.setPower(0);
                break;
            case INTAKE:
                intakeMotor.setPower(0.75);
                break;
            case REVERSE:
                intakeMotor.setPower(-0.75);
                break;
        }

    }
    public void runIntake() {
        intakeMotor.setPower(0.75);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void resetIntakeTimer() {
        intakeTimer.reset();
    }

}
