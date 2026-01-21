package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "FullCode")
public class FullRobotCode extends OpMode {

    private Follower follower;
    private Servo turret;

    private static final double TARGET_X = 136;
    private static final double TARGET_Y = 136;
    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_RANGE_RADIANS = 3;

    private Servo kickerServo;
    private DcMotorEx launcherMotor;
    private DcMotor intakeMotor;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private double targetCloseVelocity = 1150;
    private double targetFarVelocity = 1430;

    // --- Kicker state machine ---
    private enum KickerState { IDLE, EXTENDING, RETRACTING }
    private KickerState kickerState = KickerState.IDLE;
    private long stateStartTime = 0; // milliseconds
    private boolean kickerActive = false;

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        turret = hardwareMap.get(Servo.class, "turret");

        // Optional: starting pose if you want
        follower.setPose(new Pose(135, 9, Math.toRadians(90)));
        // --- Hardware init ---
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // --- Flywheel setup ---
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                //new PIDFCoefficients(2, 0, 0.0001, 19.6)
                new PIDFCoefficients(1600, 0, 0.02, 90)
        );

        // --- Servo init ---
        kickerServo.setPosition(0);

        // --- Webcam init ---

        telemetry.addLine("Init complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();

        // --- Flywheel control ---
        if (gamepad2.left_bumper) {
            launcherMotor.setVelocity(targetCloseVelocity);
        } else if (gamepad2.right_bumper) {
            launcherMotor.setVelocity(0);
        }

        // --- Intake control ---
        if (gamepad2.a) {
            intakeMotor.setPower(-1);
        } else if (gamepad2.b) {
            intakeMotor.setPower(0);
        }

        // --- Manual kicker control ---
        if (gamepad2.x) { // start kicking
            kickerActive = true;
            if (kickerState == KickerState.IDLE) {
                kickerState = KickerState.EXTENDING;
                kickerServo.setPosition(0.7);
                stateStartTime = currentTime;
            }
        }
        if (gamepad2.y) { // stop kicking
            kickerActive = false;
            kickerState = KickerState.IDLE;
            kickerServo.setPosition(0);
        }

        // --- Kicker state machine ---
        if (kickerActive) {
            switch (kickerState) {
                case EXTENDING:
                    if (currentTime - stateStartTime >= 150) { // 250ms extend
                        kickerState = KickerState.RETRACTING;
                        kickerServo.setPosition(0);
                        stateStartTime = currentTime;
                    }
                    break;

                case RETRACTING:
                    if (currentTime - stateStartTime >= 750) { // 750ms wait
                        kickerState = KickerState.EXTENDING;
                        kickerServo.setPosition(0.7);
                        stateStartTime = currentTime;
                    }
                    break;

                case IDLE:
                    // do nothing
                    break;
            }
        }

        follower.update();

        Pose robotPose = follower.getPose();

        double dx = TARGET_X - robotPose.getX();
        double dy = TARGET_Y - robotPose.getY();

        double targetAngleField = Math.atan2(dy, dx);

        double turretYaw = angleWrap(targetAngleField - robotPose.getHeading());

        double servoPos = SERVO_CENTER + (turretYaw / SERVO_RANGE_RADIANS) ;

        servoPos = clamp(servoPos, 0.1, 0.9);

        turret.setPosition(servoPos);

        telemetry.addData("Robot X", robotPose.getX());
        telemetry.addData("Robot Y", robotPose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Turret Yaw (deg)", Math.toDegrees(turretYaw));
        telemetry.addData("Servo Pos", servoPos);


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        // --- Telemetry ---
        telemetry.addData("Target Vel", targetFarVelocity);
        telemetry.addData("Actual Vel", launcherMotor.getVelocity());
        telemetry.addData("Power", launcherMotor.getPower());
        telemetry.addData("Kicker Pos", kickerServo.getPosition());
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Kicker Active", kickerActive);
        telemetry.update();


    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
