package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Kicker {

    public enum KickerState { IDLE, DOWN, UP }

    private KickerState state = KickerState.IDLE;

    private Servo kicker;
    private ElapsedTime timer = new ElapsedTime();

    private int kicks = 0;

    private final double UP_POS = 1.0;
    private final double DOWN_POS = 0.0;

    private final double MOVE_TIME = 0.35; // seconds between moves

    public void init(HardwareMap hardwareMap) {
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(DOWN_POS);
    }


    public void startKicking() {
        if (state == KickerState.IDLE) {
            kicks = 0;
            state = KickerState.UP;
            timer.reset();
        }
    }



    public void update() {
        if (state == KickerState.IDLE) return;

        if (timer.seconds() < MOVE_TIME) return;

        timer.reset();

        switch (state) {
            case UP:
                kicker.setPosition(UP_POS);
                state = KickerState.DOWN;
                break;

            case DOWN:
                kicker.setPosition(DOWN_POS);
                kicks++;

                if (kicks >= 3) {
                    state = KickerState.IDLE;
                } else {
                    state = KickerState.UP;
                }
                break;
        }
    }

    public void checkIfReady() {

    }

    public boolean isBusy() {
        return state != KickerState.IDLE;
    }
}
