package org.firstinspires.ftc.teamcode.teleop.fullcodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "fullRobotCodeStudio2")
public class fullRobotCodeStudio2 extends LinearOpMode {

    // --- Hardware ---
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotorEx rightLauncherMotor, leftLauncherMotor;
    private DcMotor intakeMotor;

    // --- Launcher States ---
    private enum LauncherState {
        OFF, LOW, MEDIUM, HIGH, FULL, CUSTOM
    }

    private LauncherState launcherState = LauncherState.OFF;

    // --- Intake States ---
    private enum IntakeState {
        OFF, INTAKE, REVERSE
    }

    private IntakeState intakeState = IntakeState.OFF;

    // --- Launcher Velocities (ticks per second) ---
    // You will need to tune these based on your specific launcher motors/gearing
    // --- Launcher Velocities (ticks per second) ---
// Adjust based on your gearbox ratio
    private static final double TICKS_PER_REV = 28; // Example: 25:1 gearbox
    private static final double RPM_TO_TICKS_PER_SEC = TICKS_PER_REV / 60.0;

    // Desired launcher wheel RPMs
    private static final double RPM_LOW = 1000;
    private static final double RPM_MEDIUM = 2000;
    private static final double RPM_HIGH = 3000;
    private static final double RPM_FULL = 4000;
    private static final double RPM_CUSTOM = 1375;

    // Converted to ticks per second
    private static final double LAUNCHER_VELOCITY_OFF = 0;
    private static final double LAUNCHER_VELOCITY_LOW = RPM_LOW * RPM_TO_TICKS_PER_SEC;
    private static final double LAUNCHER_VELOCITY_MEDIUM = RPM_MEDIUM * RPM_TO_TICKS_PER_SEC;
    private static final double LAUNCHER_VELOCITY_HIGH = RPM_HIGH * RPM_TO_TICKS_PER_SEC;
    private static final double LAUNCHER_VELOCITY_FULL = RPM_FULL * RPM_TO_TICKS_PER_SEC;
    private static final double LAUNCHER_VELOCITY_CUSTOM = RPM_CUSTOM * RPM_TO_TICKS_PER_SEC;

    @Override
    public void runOpMode() {

        // Initialize hardware
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        rightLauncherMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");
        leftLauncherMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Set motor directions
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLauncherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake behavior
        for (DcMotor m : new DcMotor[]{frontRight, backRight, frontLeft, backLeft, intakeMotor}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        leftLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized — Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateDrive();
            updateLauncher();
            updateIntake();
            updateTelemetry();
        }
    }

    // --- Drive Control ---
    private void updateDrive() {
        double y = -gamepad1.left_stick_y; // forward/backward
        double s = gamepad1.left_stick_x * 1.1; // strafe
        double x = gamepad1.right_stick_x; // turn

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(s), 1);
        double frontLeftPower = (y + x + s) / denominator;
        double backLeftPower = (y + x - s) / denominator;
        double frontRightPower = (y - x - s) / denominator;
        double backRightPower = (y - x + s) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    // --- Launcher State Machine ---
    private void updateLauncher() {
        // --- State Transitions ---
        if (gamepad2.right_bumper) {
            launcherState = LauncherState.OFF;
        } else if (gamepad2.dpad_up) {
            launcherState = LauncherState.LOW;
        } else if (gamepad2.dpad_right) {
            launcherState = LauncherState.MEDIUM;
        } else if (gamepad2.dpad_down) {
            launcherState = LauncherState.HIGH;
        } else if (gamepad2.dpad_left) {
            launcherState = LauncherState.FULL;
        } else if (gamepad2.left_bumper) {
            launcherState = LauncherState.CUSTOM;
        }

        // --- State Actions ---
        switch (launcherState) {
            case OFF:
                setLauncherVelocity(LAUNCHER_VELOCITY_OFF);
                break;
            case LOW:
                setLauncherVelocity(LAUNCHER_VELOCITY_LOW);
                break;
            case MEDIUM:
                setLauncherVelocity(LAUNCHER_VELOCITY_MEDIUM);
                break;
            case HIGH:
                setLauncherVelocity(LAUNCHER_VELOCITY_HIGH);
                break;
            case FULL:
                setLauncherVelocity(LAUNCHER_VELOCITY_FULL);
                break;
            case CUSTOM:
                setLauncherVelocity(LAUNCHER_VELOCITY_CUSTOM);
                break;
        }
    }

    private void setLauncherVelocity(double ticksPerSecond) {
        rightLauncherMotor.setVelocity(ticksPerSecond);
        leftLauncherMotor.setVelocity(ticksPerSecond);
    }

    // --- Intake State Machine ---
    private void updateIntake() {
        // --- State Transitions ---
        if (gamepad2.a) {
            intakeState = IntakeState.OFF;
        } else if (gamepad2.y || gamepad2.right_trigger > 0.05) {
            intakeState = IntakeState.INTAKE;
        } else if (gamepad2.b || gamepad2.left_trigger > 0.05) {
            intakeState = IntakeState.REVERSE;
        }

        // --- State Actions ---
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

    // --- Telemetry ---
    private void updateTelemetry() {
        telemetry.addData("Launcher State", launcherState);
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Left Launcher Velocity", leftLauncherMotor.getVelocity());
        telemetry.addData("Right Launcher Velocity", rightLauncherMotor.getVelocity());
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.update();
    }
}
