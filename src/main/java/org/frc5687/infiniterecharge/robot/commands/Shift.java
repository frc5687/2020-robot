package org.frc5687.infiniterecharge.robot.commands;


import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsytems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsytems.Shifter;

public class Shift extends OutliersCommand {
    private Shifter _shifter;
    private DriveTrain _driveTrain;
    private Shifter.Gear gear;

    private double initialLeftSpeed, initialRightSpeed;
    private long endTime;
    private State state = State.STOP_MOTOR;
    private boolean auto;

    public Shift(DriveTrain driveTrain, Shifter shifter, Shifter.Gear gear, boolean auto) {
        _driveTrain = driveTrain;
        _shifter = shifter;

        addRequirements(_driveTrain, _shifter);

        this.gear = gear;
        this.auto = auto;
    }

    @Override
    public void initialize() {
        info("Shifting to " + gear);
        state =   State.STOP_MOTOR;
    }

    @Override
    public void execute() {
        switch (state) {
            case STOP_MOTOR:
                _driveTrain.pauseMotors();
                endTime = System.currentTimeMillis() + Constants.Shifter.STOP_MOTOR_TIME;
                state = State.WAIT_FOR_MOTOR;
                break;
            case WAIT_FOR_MOTOR:
                if (System.currentTimeMillis() >= endTime) state = State.SHIFT;
                break;
            case SHIFT:
                _shifter.shift(gear, auto);
                endTime = System.currentTimeMillis() + Constants.Shifter.SHIFT_TIME;
                state = State.WAIT_FOR_SHIFT;
                break;
            case WAIT_FOR_SHIFT:
                if (System.currentTimeMillis() >= endTime) state = State.START_MOTOR;
                break;
            case START_MOTOR:
                _driveTrain.resumeMotors();
                state = State.DONE;
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    public enum State {
        STOP_MOTOR,
        WAIT_FOR_MOTOR,
        SHIFT,
        WAIT_FOR_SHIFT,
        START_MOTOR,
        DONE;
    }
}
