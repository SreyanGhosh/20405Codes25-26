package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {

    private Servo transferServo;
    private final double servoInitPos = 0.0;
    private boolean toggleState = false;
    private boolean lastButtonState = false;

    public void init(HardwareMap hardwareMap) {
        transferServo = hardwareMap.servo.get("transfer");
        transferServo.setPosition(servoInitPos); // Close Door
    }

    public void update() {
        boolean buttonState = gamepad2.b;

        if (buttonState && !lastButtonState) {
            toggleState = !toggleState;
            transferServo.setPosition(toggleState ? 0.67 : 0.0); // Immediately set position to 1 or 0
        }

        lastButtonState = buttonState;
    }

    public void setStopperPosition(double position) {
        transferServo.setPosition(position);
    }

    public void close() {
        transferServo.setPosition(0);
    }
    public void open() {
        transferServo.setPosition(0.67);
    }
}
