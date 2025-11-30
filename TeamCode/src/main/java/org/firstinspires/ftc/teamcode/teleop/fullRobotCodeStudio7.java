package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.FieldCentric;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Stopper;

@TeleOp(name = "fullRobotCodeStudio7")
public class fullRobotCodeStudio7 extends OpMode {

    FieldCentric fieldCentric = new FieldCentric();
    Launcher launcher = new Launcher();
    Intake intake = new Intake();
    Stopper stopper = new Stopper();

    @Override
    public void init() {
        fieldCentric.init(hardwareMap);
        launcher.init(hardwareMap);
        intake.init(hardwareMap);
        stopper.init(hardwareMap);
    }

    @Override
    public void loop() {
        fieldCentric.update();
        launcher.update();
        intake.update();
        stopper.update();
    }
}