package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class ExtendElevator extends OutliersCommand {
    private Climber _climber;

    @Override
    public void initialize() {
        super.initialize();
    }

    public ExtendElevator(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute() {
        _climber.setElevatorSpeed(Constants.Climber.ELEVATOR_EXTEND_SPEED);
        _climber.setWinchSpeed(Constants.Climber.WINCH_TENSION_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

