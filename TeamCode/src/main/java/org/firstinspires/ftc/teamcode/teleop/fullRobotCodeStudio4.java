package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "fullRobotCodeStudio4")
public class fullRobotCodeStudio4 extends LinearOpMode {
    // --- Hardware ---
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotorEx rightLauncherMotor, leftLauncherMotor;
    private DcMotor intakeMotor;

    private Servo transferServo;
    private double servoInitPos = 0.0;
    private boolean toggleState = false;
    private boolean lastButtonState = false;

    // --- PIDF Coefficients for Launcher Consistency ---
    private static final double kP = 5.1;  // Proportional (adjust as needed)
    private static final double kI = 0.0;   // Integral
    private static final double kD = 0.5;   // Derivative
    private static final double kF = 19.2;  // Feedforward (adjust for your motor)


    // --- Launcher States ---
    private enum LauncherState {
        OFF, SPIN_WHEEL, LAUNCH, PREP_NEXT, REVERSE
    }

    private enum IntakeState {
        OFF, INTAKE, REVERSE
    }

    private LauncherState launcherState = LauncherState.OFF;
    private int numOfBallsToLaunch = 1;
    // --- Intake States ---

    private IntakeState intakeState = IntakeState.OFF;
    // --- Launcher Velocities (ticks per second) ---
    // You will need to tune these based on your specific launcher motors/gearing
    // --- Launcher Velocities (ticks per second) ---
    // Adjust based on your gearbox ratio
    private static final double TICKS_PER_REV = 28; // Example: 25:1 gearbox
    private static final double RPM_TO_TICKS_PER_SEC = TICKS_PER_REV / 60.0;
    // Desired launcher wheel RPMs
    private static double DEFAULT_RPM = 1359;
    private static double CURRENT_RPM = DEFAULT_RPM;
    private static double RPM_INCREMENT = 20;
    // Converted to ticks per second
    private static final double LAUNCHER_OFF_VELOCITY = 0;
    private static final double SAFETY_CONSTANT = 0.99;
    private static final double LAUNCHER_MIN_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC * SAFETY_CONSTANT;
    private static double LAUNCHER_TARGET_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC;

    private ElapsedTime intakeTimer = new ElapsedTime();
    private boolean intakeTimerStarted = false;
    private ElapsedTime launcherTimer = new ElapsedTime();
    private boolean launcherTimerStarted = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        transferServo = hardwareMap.servo.get("transfer");
        transferServo.setPosition(servoInitPos); // Close Door

        // Launcher Motors
        rightLauncherMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");
        leftLauncherMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");

// Brake behavior
        leftLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// Use encoder and apply PIDF control
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));
        leftLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));

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

            CURRENT_RPM = Math.max(CURRENT_RPM, 0);

            updateDrive();

            boolean buttonState = gamepad2.b;

            if (buttonState && !lastButtonState) {
                toggleState = !toggleState;
                transferServo.setPosition(toggleState ? 0.67 : 0.0); // Immediately set position to 1 or 0
            }

            lastButtonState = buttonState;


            // Change the launcher RPM with the DPAD UP or DOWN by 50rpm value
            if (gamepad2.dpad_up) {
                CURRENT_RPM += RPM_INCREMENT;
                LAUNCHER_TARGET_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC;
            } else if (gamepad2.dpad_down) {
                CURRENT_RPM -= RPM_INCREMENT;
                LAUNCHER_TARGET_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC;
            }
            if (gamepad2.y) {
                launcherState = LauncherState.REVERSE;
            }
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

        switch (launcherState) {
            case REVERSE:
                launcherTimer.reset();
                if (launcherTimer.milliseconds() <= 1000) {
                    rightLauncherMotor.setPower(-0.2);
                    leftLauncherMotor.setPower(0.2);
                } else if (launcherTimer.milliseconds() >= 1000) {
                    launcherState = LauncherState.OFF;
                }
                break;
            case OFF:
                if (gamepad2.left_bumper || gamepad2.right_bumper) {
                    transferServo.setPosition(0); // Close door
                    launcherState = LauncherState.SPIN_WHEEL;
                    if (gamepad2.right_bumper) {
                        numOfBallsToLaunch = 2;
                    }
                }
                break;
            case SPIN_WHEEL:
                transferServo.setPosition(0.67); // Open door
                setLauncherVelocity(LAUNCHER_TARGET_VELOCITY);
                if (getLauncherVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launcherState = LauncherState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (!intakeTimerStarted) {
                    intakeTimer.reset();
                    intakeTimerStarted = true;
                }

                if (intakeTimer.milliseconds() < 100) {
                    intakeMotor.setPower(0.75); // feed ring
                } else if (intakeTimer.milliseconds() < 150) {
                    intakeMotor.setPower(0);
                    transferServo.setPosition(0); // close flap
                } else {
                    intakeTimerStarted = false;
                    launcherState = LauncherState.PREP_NEXT;
                }
                break;
            case PREP_NEXT:
                if (!launcherTimerStarted) {
                    launcherTimer.reset();
                    launcherTimerStarted = true;
                }

                if (launcherTimer.milliseconds() >= 200) { // wait before next feed
                    launcherTimerStarted = false;
                    if (numOfBallsToLaunch > 1) {
                        numOfBallsToLaunch--;
                        launcherState = LauncherState.LAUNCH;
                    } else {
                        setLauncherVelocity(LAUNCHER_OFF_VELOCITY);
                        launcherState = LauncherState.OFF;
                    }
                }
                break;
        }
    }
    private void setLauncherVelocity(double ticksPerSecond) {
        rightLauncherMotor.setVelocity(ticksPerSecond);
        leftLauncherMotor.setVelocity(ticksPerSecond);
    }

    // Returns the average velocity of the launcher wheels in ticks per second
    private double getLauncherVelocity() {
        return (rightLauncherMotor.getVelocity() + leftLauncherMotor.getVelocity()) / 2;
    }
    // --- Intake State Machine ---
    private void updateIntake() {
        // --- State Transitions ---
        if (gamepad2.a) {
            intakeState = IntakeState.OFF;
        } else if (gamepad2.right_trigger > 0.05) {
            intakeState = IntakeState.INTAKE;
        } else if (gamepad2.left_trigger > 0.05) {
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
        telemetry.addData("Target RPM", LAUNCHER_TARGET_VELOCITY);
        telemetry.update();
    }
}