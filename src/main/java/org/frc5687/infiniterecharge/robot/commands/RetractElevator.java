package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class RetractElevator extends OutliersCommand {
    private Climber _climber;

    @Override
    public void initialize() {
        super.initialize();
    }

    public RetractElevator(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void execute() {
        _climber.setElevatorSpeed(Constants.Climber.ELEVATOR_RETRACT_SPEED);
        // _climber.setWinchSpeed(Constants.Climber.WINCH_TENSION_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

