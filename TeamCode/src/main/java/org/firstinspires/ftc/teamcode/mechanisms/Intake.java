package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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

        for (DcMotor m : new DcMotor[]{intakeMotor}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void runFor(double time) {
        resetIntakeTimer();
        if(intakeTimer.milliseconds() < time) {
            intakeMotor.setPower(0.75);
        }
        else {
            off();
        }

    }
    public void run() {
        intakeMotor.setPower(0.75);
    }


    public void off() {
        intakeMotor.setPower(0);
    }

    public void update(Gamepad gamepad2) {
        if (gamepad2.right_bumper) {
            run();
        } else if(gamepad2.left_bumper) {
            off();
        }
    }

    public void resetIntakeTimer() {
        intakeTimer.reset();
    }

}
