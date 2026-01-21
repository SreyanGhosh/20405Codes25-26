package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "FullRobotTeleOp")
public class FullRobotTeleOp extends OpMode {

    // ---- Subsystems ----
    private DriveSubsystem drive;
    private FollowerSubsystem follower;
    private TurretSubsystem turret;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private KickerSubsystem kicker;

    // ---- Shooter ----
    private double targetVelocity = 1500;
    private boolean lastLB = false;

    // ---- Turret target ----
    private static final double TARGET_X = 136;
    private static final double TARGET_Y = 134;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap);
        follower = new FollowerSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "launcherMotor")
        );
        intake = new IntakeSubsystem(hardwareMap);
        kicker = new KickerSubsystem(hardwareMap);

        telemetry.addLine("Full Robot TeleOp Ready");
        telemetry.update();
    }

    @Override
    public void loop() {



        double dist = follower.getDistanceTo(TARGET_X, TARGET_Y);


        // ---- Localization ----
        follower.update();

        // ---- Turret auto-aim ----
        turret.aimAt(
                follower.getPose(),
                TARGET_X,
                TARGET_Y
        );


        boolean lb = gamepad2.left_bumper;
        boolean togglePressed = lb && !lastLB;
        lastLB = lb;

        shooter.update(togglePressed, targetVelocity);

        // ---- Intake ----
        intake.run(gamepad2.a);

        // ---- Kicker ----
        if (gamepad2.x) {
            kicker.start();
            intake.runIntake();
        }
        if (gamepad2.y) {
            kicker.stop();
            intake.stop();
        }
        kicker.update();

        // ---- Drive ----
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        drive.drive(y, x, rx);

        // ---- Telemetry ----
        telemetry.addData("Shooter ON", shooter.isEnabled());
        telemetry.addData("Target RPM", targetVelocity);
        telemetry.addData("Distance from Target", dist);
        telemetry.update();
    }

}
