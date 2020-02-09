package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class RetractWinch extends OutliersCommand {
    private Climber _climber;

    public RetractWinch(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _climber.setWinchSpeed(Constants.Climber.WINCH_RETRACT_SPEED);
        _climber.setElevatorSpeed(Constants.Climber.ELEVATOR_TENSION_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

