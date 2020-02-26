package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;

public class DriveHood extends OutliersCommand {
    private Hood _hood;
    private OI _oi;

    private boolean _zeroing = false;

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
        if (_zeroing) {
            _hood.setSpeed(Constants.Hood.ZEROING_SPEED);
            if (_hood.isHallTriggered()) {
                _hood.setPosition(Constants.Hood.MIN_DEGREES);
                _zeroing = false;
            }
        } else {
            double speed = _oi.getHoodSpeed();
            _hood.setPosition(_hood.getSetPoint() + speed * Constants.Hood.SENSITIVITY);
        }
    }

    public void setZeroing(boolean value) {
        _zeroing = true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
