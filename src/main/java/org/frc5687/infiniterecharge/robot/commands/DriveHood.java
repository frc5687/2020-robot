package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;

public class DriveHood extends OutliersCommand {
    private Hood _hood;
    private OI _oi;

    public DriveHood(Hood hood, OI oi) {
        _hood = hood;
        _oi = oi;
        addRequirements(_hood);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        double speed = _oi.getHoodSpeed();
        _hood.setSpeed(speed);
         _hood.setPosition(_hood.getSetPoint() + speed * Constants.Hood.SENSITIVITY);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
