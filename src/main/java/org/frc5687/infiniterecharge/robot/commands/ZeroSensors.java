package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;

public class ZeroSensors extends OutliersCommand {
    private Hood _hood;
    private Turret _turret;

    public ZeroSensors(Hood hood, Turret turret) {
        _hood = hood;
        _turret = turret;
        addRequirements(_hood);
    }

    @Override
    public void initialize() {
        super.initialize();
        _turret.zeroSensors();
        error("Zeroing Hood");
    }

    @Override
    public void execute() {
        super.execute();
        _hood.setSpeed(Constants.Hood.ZEROING_SPEED);
    }

    @Override
    public boolean isFinished() {
        error("is Finishing");
        return _hood.isHallTriggered(); }

}
