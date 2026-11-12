package org.firstinspires.ftc.teamcode.subsystems.gate;

import static org.firstinspires.ftc.teamcode.subsystems.gate.GateConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.TauraServo;

public class Gate extends SubsystemBase {
    public enum State {
        OPEN, CLOSED
    }

    private final TauraServo servo;
    private State state = State.CLOSED;

    public Gate(HardwareMap hardwareMap) {
        servo = new TauraServo(hardwareMap.get(Servo.class, SERVO_NAME));
    }

    public void setState(State state) {
        this.state = state;
        apply();
    }

    public State getState() {
        return state;
    }

    public boolean isOpen() {
        return state == State.OPEN;
    }

    public void open() {
        setState(State.OPEN);
    }

    public void close() {
        setState(State.CLOSED);
    }

    public void toggle() {
        setState(isOpen() ? State.CLOSED : State.OPEN);
    }

    private void apply() {
        servo.setDirection(SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo.setPosition(state == State.OPEN ? OPEN_POSITION : CLOSED_POSITION);
    }

    @Override
    public void periodic() {
        apply();
    }

    public void addTelemetry(TelemetryData telemetry) {
        telemetry.addData("Gate state", state);
        telemetry.addData("Gate position", servo.getPosition());
    }
}
