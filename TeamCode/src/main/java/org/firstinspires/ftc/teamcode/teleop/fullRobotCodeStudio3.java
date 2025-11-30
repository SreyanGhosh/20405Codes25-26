package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "fullRobotCodeStudio3")
public class fullRobotCodeStudio3 extends LinearOpMode {
    // --- Hardware ---
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotorEx rightLauncherMotor, leftLauncherMotor;
    private DcMotor intakeMotor;

    private Servo transferServo;
    private double servoInitPos = 0.67;
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
    private static double DEFAULT_RPM = 1355;
    private static double CURRENT_RPM = DEFAULT_RPM;
    private static double RPM_INCREMENT = 50;
    // Converted to ticks per second
    private static final double LAUNCHER_OFF_VELOCITY = 0;
    private static final double LAUNCHER_MIN_VELOCITY = 1355 * RPM_TO_TICKS_PER_SEC;
    private static double LAUNCHER_TARGET_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC;

    private ElapsedTime feederTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        transferServo = hardwareMap.servo.get("transfer");
        transferServo.setPosition(0.0); // Close Door

        // Launcher Motors
        rightLauncherMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");
        leftLauncherMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");

// Set directions
        leftLauncherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // Launcher States
        // 1. OFF - Default state where the launcher is not spinning.
        //          When Gamepad2.left_bumper or Gamepad2.right_launcher is pressed, change to SPIN_WHEEL.
        //          Use Gamepad2.left_bumper to launch 1 ball, and Gamepad2.right_launcher to launch 2 balls.
        // 2. SPIN_WHEEL - Start spinning the launcher wheel and wait for it to reach a min. velocity.
        //          Change to LAUNCH_FIRST when the launcher wheel reaches a min. velocity.
        // 3. LAUNCH_FIRST - Start the intake motor, reset the feeder timer, and change to LAUNCHING state.
        // 4. LAUNCHING - Wait for the feeder timer to reach 0.5 seconds. (This timer needs tweaking).
        //          Stop the intake motor.
        //          If second ball needs to be launched, change to LAUNCH_SECOND. Sleep for a time (time needs tweaking).
        //          If no second ball needs to be launched, change to OFF.
        // 5. LAUNCH_SECOND - Start the intake motor, reset the feeder timer, and change to LAUNCHING state.
        // 6. LAUNCHING - Wait for the feeder timer to reach 0.5 seconds. (This timer needs tweaking).
        //          Stop the intake motor. Change to OFF.
        // 7. OFF

        switch (launcherState) {
            case REVERSE:
                rightLauncherMotor.setPower(-0.2);
                leftLauncherMotor.setPower(0.2);
                sleep(1000);
                rightLauncherMotor.setPower(0);
                leftLauncherMotor.setPower(0);
                intakeMotor.setPower(0);
                transferServo.setPosition(0); // Close door
                launcherState = LauncherState.OFF;
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
                if (getLauncherVelocity() > LAUNCHER_TARGET_VELOCITY) {
                    launcherState = LauncherState.LAUNCH;
                }
                break;
            case LAUNCH:
                // Use the intake motors to bring the ball closer to the door
                // Open the door & launch
                // Close the door
                // intakeMotor.setPower(0.75); sleep(100); intakeMotor.setPower(0);
                intakeMotor.setPower(0.75);
                sleep(100);
                intakeMotor.setPower(0);
                sleep(100);
                transferServo.setPosition(0); // Close door
                launcherState = LauncherState.PREP_NEXT;
                break;
            case PREP_NEXT:
                if (numOfBallsToLaunch > 1) {      // Wait and launch the next ball
                    sleep(100);
                    numOfBallsToLaunch--;
                    launcherState = LauncherState.LAUNCH;
                } else {
                    launcherState = LauncherState.OFF;
                    setLauncherVelocity(LAUNCHER_OFF_VELOCITY);
                }
                break;
        }
    }
    private void setLauncherVelocity(double ticksPerSecond) {
        rightLauncherMotor.setVelocity(ticksPerSecond);
        leftLauncherMotor.setVelocity(ticksPerSecond * 1.05567);
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