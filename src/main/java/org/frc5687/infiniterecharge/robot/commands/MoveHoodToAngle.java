package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Hood;

public class MoveHoodToAngle extends OutliersCommand {

    private Hood _hood;
    private double _angle;

    public MoveHoodToAngle(Hood hood, double angle) {
        _hood = hood;
        _angle = angle;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        _hood.setPosition(_angle);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return _hood.isAtSetpoint();
    }
}
