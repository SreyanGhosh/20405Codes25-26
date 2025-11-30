package org.firstinspires.ftc.teamcode.mechanisms;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    private DcMotorEx rightLauncherMotor, leftLauncherMotor;
    Intake intake = new Intake();
    Stopper stopper = new Stopper();


    private static final double kP = 5.1;
    private static final double kI = 0.0;
    private static final double kD = 0.5;
    private static final double kF = 19.2;


    // --- Launcher States ---
    private enum LauncherState {
        OFF, SPIN_WHEEL, LAUNCH, PREP_NEXT, REVERSE
    }

    private LauncherState launcherState = LauncherState.OFF;
    private int numOfBallsToLaunch = 1;

    private static final double TICKS_PER_REV = 28;
    private static final double RPM_TO_TICKS_PER_SEC = TICKS_PER_REV / 60.0;
    // Desired launcher wheel RPMs
    private static final double DEFAULT_RPM = 1359;
    private static double CURRENT_RPM = DEFAULT_RPM;
    private static final double RPM_INCREMENT = 20;
    // Converted to ticks per second
    private static final double LAUNCHER_OFF_VELOCITY = 0;
    private static final double SAFETY_CONSTANT = 0.99;
    private static double LAUNCHER_MIN_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC * SAFETY_CONSTANT;
    private static double LAUNCHER_TARGET_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC;
    private final ElapsedTime launcherTimer = new ElapsedTime();
    private boolean launcherTimerStarted = false;


    public void init(HardwareMap hardwareMap) {

        rightLauncherMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");
        leftLauncherMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");

        leftLauncherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));
        leftLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));
    }

    public void setLauncherVelocity(double ticksPerSecond) {
        rightLauncherMotor.setVelocity(ticksPerSecond);
        leftLauncherMotor.setVelocity(ticksPerSecond);
    }

    private double getLauncherVelocity() {
        return (rightLauncherMotor.getVelocity() + leftLauncherMotor.getVelocity()) / 2;
    }

    public void update() {

        if (gamepad2.dpad_up) {
            CURRENT_RPM += RPM_INCREMENT;
            LAUNCHER_TARGET_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC;
            LAUNCHER_MIN_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC * SAFETY_CONSTANT;
        } else if (gamepad2.dpad_down) {
            CURRENT_RPM -= RPM_INCREMENT;
            LAUNCHER_TARGET_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC;
            LAUNCHER_MIN_VELOCITY = CURRENT_RPM * RPM_TO_TICKS_PER_SEC * SAFETY_CONSTANT;
        }

        if (gamepad2.y) {
            launcherState = LauncherState.REVERSE;
        }
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
                    stopper.close(); // Close door
                    launcherState = LauncherState.SPIN_WHEEL;
                    if (gamepad2.right_bumper) {
                        numOfBallsToLaunch = 2;
                    }
                }
                break;
            case SPIN_WHEEL:
                stopper.open();
                setLauncherVelocity(LAUNCHER_TARGET_VELOCITY);
                if (getLauncherVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launcherState = LauncherState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (!intake.intakeTimerStarted) {
                    intake.resetIntakeTimer();
                    intake.intakeTimerStarted = true;
                }

                if (intake.IntakeTimer < 100) {
                    intake.runIntake();
                } else if (intake.IntakeTimer < 150) {
                    intake.stopIntake();
                    stopper.close();
                } else {
                    intake.intakeTimerStarted = false;
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
}
