package org.firstinspires.ftc.teamcode.teleop.fullcodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TurretOpMode")
public class TurretOpMode extends OpMode {

    /* ===================== PEDRO ===================== */
    private Follower follower;

    /* ===================== TURRET HARDWARE ===================== */
    private CRServo turretServo;
    private DcMotor turretEncoder;

    /* ===================== ENCODER ===================== */
    private static final double ENCODER_TICKS_PER_REV = 8192.0;
    private static final double TURRET_GEAR_RATIO = 1.0;

    /* ===================== CONTROL GAINS ===================== */
    private static final double kP = 1.8;      // start here
    private static final double kD = 0.15;
    private static final double MAX_POWER = 0.7;
    private static final double DEADZONE_RAD = Math.toRadians(0.5);

    private double lastError = 0;

    /* ===================== HEIGHTS ===================== */
    private static final double TURRET_HEIGHT = 9.0;   // inches
    private static final double TARGET_Z = 29.5;

    /* ===================== TARGET (PEDRO COORDS) ===================== */
    // FTC: X = -58.3727, Y = 55.6425
    private static final double TARGET_X = 127.6425;
    private static final double TARGET_Y = 13.6273;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        turretEncoder = hardwareMap.get(DcMotor.class, "turretEncoder");

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretServo.setPower(0);
    }

    @Override
    public void loop() {

        follower.update();
        updateTurretAimClosedLoop();

        telemetry.addData("Turret Angle (deg)",
                Math.toDegrees(getTurretAngleRad()));
        telemetry.addData("Turret Power", turretServo.getPower());
    }

    /* ===================== CLOSED-LOOP TURRET ===================== */
    private void updateTurretAimClosedLoop() {

        Pose pose = follower.getPose();

        /* --- 3D geometry --- */
        double dx = TARGET_X - pose.getX();
        double dy = TARGET_Y - pose.getY();
        double dz = TARGET_Z - TURRET_HEIGHT; // reserved for pitch

        /* --- yaw math --- */
        double fieldYaw = Math.atan2(dy, dx);
        double desiredYaw =
                angleWrap(fieldYaw - pose.getHeading());

        double currentYaw = getTurretAngleRad();
        double error = angleWrap(desiredYaw - currentYaw);

        /* --- deadzone --- */
        if (Math.abs(error) < DEADZONE_RAD) {
            turretServo.setPower(0);
            return;
        }

        /* --- PD control --- */
        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kD * derivative;
        power = clip(power, -MAX_POWER, MAX_POWER);

        turretServo.setPower(power);
    }

    /* ===================== ENCODER ===================== */
    private double getTurretAngleRad() {
        return (turretEncoder.getCurrentPosition()
                / ENCODER_TICKS_PER_REV)
                * 2.0 * Math.PI / TURRET_GEAR_RATIO;
    }

    /* ===================== UTIL ===================== */
    private double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
