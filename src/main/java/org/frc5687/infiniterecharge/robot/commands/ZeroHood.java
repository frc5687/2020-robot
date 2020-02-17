package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;

public class ZeroHood extends OutliersCommand {
    private Hood _hood;

    public ZeroHood(Hood hood) {
        _hood = hood;
        addRequirements(_hood);
    }

    @Override
    public void initialize() {
        super.initialize();
        error("Zeroing Hood");
    }

    @Override
    public void execute() {
        super.execute();
        _hood.setSpeed(Constants.Hood.ZEROING_SPEED);
    }

    @Override
    public boolean isFinished() {
        return _hood.isHallTriggered(); }

}
