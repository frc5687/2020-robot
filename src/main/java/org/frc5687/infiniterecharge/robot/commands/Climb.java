package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;

public class Climb extends OutliersCommand {
    private OI _oi;
    private Climber _climber;
    private double _rotationSpeed;

    public Climb(Climber climber, OI oi) {
        _climber = climber;
        _oi = oi;
        addRequirements(_climber);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double climberSpeed = _oi.getClimberSpeed();
        _climber.setSpeed(climberSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

