package org.firstinspires.ftc.teamcode.teleop.fullcodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.*;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends OpMode {

    RobotCentric drivetrain = new RobotCentric();
    Flywheel flywheel = new Flywheel();
    Intake intake = new Intake();
    Kicker kicker = new Kicker();

    // Button tracking
    boolean lastB1 = false;
    boolean lastB2 = false;
    boolean lastB3 = false;

    boolean intakeOn = false;
    boolean reverseOn = false;

    enum ShootState { IDLE, SPINUP, FEEDING, KICKING, DONE }
    ShootState shootState = ShootState.IDLE;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        flywheel.init(hardwareMap);
        intake.init(hardwareMap);
        kicker.init(hardwareMap);
    }

    @Override
    public void loop() {

        drivetrain.updateDrive(gamepad1);

        // === B1: Toggle Intake ===
        if (gamepad2.a && !lastB1) {
            intakeOn = !intakeOn;
            reverseOn = false;
        }
        lastB1 = gamepad2.a;

        // === B3: Toggle Reverse Intake ===
        if (gamepad2.x && !lastB3) {
            reverseOn = !reverseOn;
            intakeOn = false;
        }
        lastB3 = gamepad2.x;

        // === Intake Control ===
        if (intakeOn) {
            intake.run();
        } else if (reverseOn) {
            // reverse intake
            intake.runFor(0.1);
            intake.off();
        } else {
            intake.off();
        }

        // === B2: Shooting Sequence ===
        if (gamepad2.b && !lastB2 && shootState == ShootState.IDLE) {
            shootState = ShootState.SPINUP;
        }
        lastB2 = gamepad2.b;

        switch (shootState) {

            case SPINUP:
                flywheel.setVelocity(flywheel.getTargetVelocity());
                if (flywheel.atSpeed(flywheel.getTargetVelocity(), 50)) {
                    shootState = ShootState.FEEDING;
                }
                break;

            case FEEDING:
                intake.run();
                kicker.startKicking();
                shootState = ShootState.KICKING;
                break;

            case KICKING:
                kicker.update();
                if (!kicker.isBusy()) {
                    intake.off();
                    flywheel.stop();
                    shootState = ShootState.DONE;
                }
                break;

            case DONE:
                shootState = ShootState.IDLE;
                break;
        }

        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Flywheel", flywheel.getVelocity());
        telemetry.addData("Target", flywheel.getTargetVelocity());
        telemetry.addData("Intake", intakeOn ? "ON" : reverseOn ? "REVERSE" : "OFF");
        telemetry.update();
    }
}
