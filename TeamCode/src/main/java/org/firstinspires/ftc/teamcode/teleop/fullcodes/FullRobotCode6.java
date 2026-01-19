package org.firstinspires.ftc.teamcode.teleop.fullcodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "FullRobotCode6")
public class FullRobotCode6 extends OpMode {

    private Servo kickerServo;
    private DcMotorEx launcherMotor;
    private DcMotor intakeMotor;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    private Webcam webcam = new Webcam();

    private double targetVelocity = 2000;

    // --- Intake toggle ---
    private boolean intakeOn = false;
    private boolean intakeTogglePressed = false;

    // --- Launch state machine ---
    private enum LaunchState { IDLE, SPINNING_UP, RUNNING_INTAKE, RUNNING_KICKER }
    private LaunchState launchState = LaunchState.IDLE;
    private boolean launchButtonPressedLast = false;

    // Kicker timing
    private enum KickerState { EXTENDING, RETRACTING }
    private KickerState kickerState = KickerState.EXTENDING;
    private long kickerTimer = 0;

    @Override
    public void init() {
        // --- Hardware init ---
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- Flywheel setup ---
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(0.00215, 0, 0.0001, 19.9)
        );

        // --- Servo init ---
        kickerServo.setPosition(0);

        // --- Webcam init ---
        webcam.init(hardwareMap, telemetry);

        telemetry.addLine("Init complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();

        // --- Intake toggle button (A) ---
        if (gamepad1.a) {
            if (!intakeTogglePressed) {
                intakeOn = !intakeOn;
                intakeTogglePressed = true;
            }
        } else {
            intakeTogglePressed = false;
        }

        // Set intake when not in launch sequence
        if (launchState == LaunchState.IDLE) {
            intakeMotor.setPower(intakeOn ? -1 : 0);
        }

        // --- Launch button (X) pressed ---
        if (gamepad1.x && !launchButtonPressedLast) {
            launchState = LaunchState.SPINNING_UP;
        }
        launchButtonPressedLast = gamepad1.x;

        // --- Stop launch button (Y) ---
        if (gamepad1.y) {
            launchState = LaunchState.IDLE;
            launcherMotor.setVelocity(0);
            intakeMotor.setPower(0);
            kickerServo.setPosition(0);
        }

        // --- Launch state machine ---
        switch (launchState) {
            case IDLE:
                // do nothing
                break;

            case SPINNING_UP:
                launcherMotor.setVelocity(targetVelocity);
                // wait until flywheel reaches 95% of target
                if (launcherMotor.getVelocity() >= targetVelocity * 0.95) {
                    launchState = LaunchState.RUNNING_INTAKE;
                    intakeMotor.setPower(-1);
                    // start kicker timer
                    kickerTimer = currentTime;
                    kickerState = KickerState.EXTENDING;
                }
                break;

            case RUNNING_INTAKE:
                // Intake already on, wait 100ms then start kicker
                if (currentTime - kickerTimer >= 100) {
                    launchState = LaunchState.RUNNING_KICKER;
                    kickerServo.setPosition(0.7);
                    kickerTimer = currentTime;
                }
                break;

            case RUNNING_KICKER:
                // Kicker timed loop
                switch (kickerState) {
                    case EXTENDING:
                        if (currentTime - kickerTimer >= 250) { // 250ms extend
                            kickerServo.setPosition(0);
                            kickerState = KickerState.RETRACTING;
                            kickerTimer = currentTime;
                        }
                        break;

                    case RETRACTING:
                        if (currentTime - kickerTimer >= 750) { // 750ms retract
                            kickerServo.setPosition(0.7);
                            kickerState = KickerState.EXTENDING;
                            kickerTimer = currentTime;
                        }
                        break;
                }
                break;
        }

        // --- Webcam / AprilTag ---
        webcam.update();
        AprilTagDetection tag = webcam.getTagBySpecificId(24);
        if (tag != null) {
            telemetry.addData("Distance (in)", tag.ftcPose.range);
        }

        // --- Telemetry ---
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Actual Vel", launcherMotor.getVelocity());
        telemetry.addData("Kicker Pos", kickerServo.getPosition());
        telemetry.addData("Intake On", intakeMotor.getPower());
        telemetry.addData("Launch State", launchState);
        telemetry.update();


        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}
