package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.RobotCentric;
import org.firstinspires.ftc.teamcode.mechanisms.Stopper;

@TeleOp(name = "fullRobotCodeStudio6")
public class fullRobotCodeStudio6 extends OpMode {

    RobotCentric robotCentric = new RobotCentric();
    Launcher launcher = new Launcher();
    Intake intake = new Intake();
    Stopper stopper = new Stopper();

    @Override
    public void init() {
        robotCentric.init(hardwareMap);
        launcher.init(hardwareMap);
        intake.init(hardwareMap);
        stopper.init(hardwareMap);
    }

    @Override
    public void loop() {
        robotCentric.update();
        launcher.update();
        intake.update();
        stopper.update();
    }
}
