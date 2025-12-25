package org.firstinspires.ftc.teamcode.teleop.fullcodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "fullRobotCodeStudio")
public class fullRobotCodeStudio1 extends LinearOpMode {

    // --- Hardware ---
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private DcMotor rightLauncherMotor, leftLauncherMotor, intakeMotor;

    // --- Launcher States ---
    private enum LauncherState {
        OFF, LOW, MEDIUM, HIGH, FULL, CUSTOM
    }

    private LauncherState launcherState = LauncherState.OFF;

    // --- Intake States ---
    private enum IntakeState {
        OFF, INTAKE, REVERSE, TRANSFER
    }

    private IntakeState intakeState = IntakeState.OFF;

    @Override
    public void runOpMode() {

        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        backRight = hardwareMap.get(DcMotorEx.class,"backRight");

        rightLauncherMotor = hardwareMap.get(DcMotor.class, "rightLauncherMotor");
        leftLauncherMotor = hardwareMap.get(DcMotor.class, "leftLauncherMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Set motor directions
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake behavior
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        double s = -gamepad1.left_stick_x * 1.1; // strafe
        double x = -gamepad1.right_stick_x; // turn



        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(s), 1);
        double frontLeftPower = ((y + x + s)) / denominator;
        double backLeftPower = ((y + x - s)) / denominator;
        double frontRightPower = ((y - x - s)) / denominator;
        double backRightPower = ((y - x + s)) / denominator;

        frontLeft.setVelocity(frontLeftPower);
        backLeft.setVelocity(backLeftPower);
        frontRight.setVelocity(frontRightPower);
        backRight.setVelocity(backRightPower);
    }

    // --- Launcher State Machine ---
    private void updateLauncher() {

        // --- State Transitions ---
        if (gamepad1.left_bumper) {
            launcherState = LauncherState.OFF;
        } else if (gamepad1.dpad_up) {
            launcherState = LauncherState.LOW;
        } else if (gamepad1.dpad_right) {
            launcherState = LauncherState.MEDIUM;
        } else if (gamepad1.dpad_down) {
            launcherState = LauncherState.HIGH;
        } else if (gamepad1.dpad_left) {
            launcherState = LauncherState.FULL;
        } else if (gamepad1.right_bumper) {
            launcherState = LauncherState.CUSTOM;
        }

        // --- State Actions ---
        switch (launcherState) {
            case OFF:
                setLauncherPower(0);
                break;
            case LOW:
                setLauncherPower(0.25);
                break;
            case MEDIUM:
                setLauncherPower(0.50);
                break;
            case HIGH:
                setLauncherPower(0.75);
                break;
            case FULL:
                setLauncherPower(1.0);
                break;
            case CUSTOM:
                setLauncherPower(0.375);
                break;
        }
    }

    private void setLauncherPower(double power) {
        rightLauncherMotor.setPower(-power);
        leftLauncherMotor.setPower(power);
    }

    // --- Intake + Servo State Machine ---
    private void updateIntake() {

        // --- State Transitions ---
        if (gamepad1.a) {
            intakeState = IntakeState.OFF;
        } else if (gamepad1.y) {
            intakeState = IntakeState.TRANSFER;
        } else if (gamepad1.b) {
            intakeState = IntakeState.REVERSE;
        } else if (gamepad1.right_trigger > 0.05) {
            intakeState = IntakeState.INTAKE;
        } else if (gamepad1.left_trigger > 0.05) {
            intakeState = IntakeState.REVERSE;
        }

        // --- State Actions ---
        switch (intakeState) {
            case OFF:
                intakeMotor.setPower(0);
                break;
            case INTAKE:
                intakeMotor.setPower(gamepad1.right_trigger);
                break;
            case REVERSE:
                intakeMotor.setPower(gamepad1.left_trigger);
                break;
            case TRANSFER:
                intakeMotor.setPower(0.75);
                break;
        }
    }

    // --- Telemetry ---
    private void updateTelemetry() {
        telemetry.addData("Launcher State", launcherState);
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Right Launcher Power", rightLauncherMotor.getPower());
        telemetry.addData("Left Launcher Power", leftLauncherMotor.getPower());
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.update();
    }
}
