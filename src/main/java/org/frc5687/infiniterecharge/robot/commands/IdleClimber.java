package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class IdleClimber extends OutliersCommand {
    private Climber _climber;

    public IdleClimber(Climber climber) {
        _climber = climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _climber.setWinchSpeed(0);
        _climber.setElevatorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
