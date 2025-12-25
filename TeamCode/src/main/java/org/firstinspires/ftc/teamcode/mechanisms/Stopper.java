package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {

    private Servo transferServo;
    private final double servoInitPos = 0.0;
    private boolean toggleState = false;
    private boolean lastButtonState = false;

    public Stopper(HardwareMap hardwareMap) {
        transferServo = hardwareMap.servo.get("transfer");
        transferServo.setPosition(servoInitPos); // Close Door
    }

    public void updateStopper(Gamepad gamepad1, Gamepad gamepad2) {
        boolean buttonState = gamepad2.b;

        if (buttonState && !lastButtonState) {
            toggleState = !toggleState;
            transferServo.setPosition(toggleState ? 0.67 : 0.0); // Immediately set position to 1 or 0
        } else if (gamepad1.y) {
            transferServo.setPosition(0.5);
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
