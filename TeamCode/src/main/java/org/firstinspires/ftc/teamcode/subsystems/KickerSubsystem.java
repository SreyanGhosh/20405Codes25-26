package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class KickerSubsystem {

    private enum State { IDLE, EXTEND, RETRACT }

    private final Servo kicker;
    private State state = State.IDLE;
    private long stateTime = 0;
    private boolean active = false;

    public KickerSubsystem(HardwareMap hw) {
        kicker = hw.get(Servo.class, "kickerServo");
        kicker.setPosition(0);
    }

    public void start() {
        active = true;
        if (state == State.IDLE) {
            state = State.EXTEND;
            kicker.setPosition(0.7);
            stateTime = System.currentTimeMillis();
        }
    }

    public void stop() {
        active = false;
        state = State.IDLE;
        kicker.setPosition(0);
    }

    public void update() {
        if (!active) return;

        long now = System.currentTimeMillis();

        switch (state) {
            case EXTEND:
                if (now - stateTime > 150) {
                    state = State.RETRACT;
                    kicker.setPosition(0);
                    stateTime = now;
                }
                break;

            case RETRACT:
                if (now - stateTime > 750) {
                    state = State.EXTEND;
                    kicker.setPosition(0.7);
                    stateTime = now;
                }
                break;
        }
    }
}
