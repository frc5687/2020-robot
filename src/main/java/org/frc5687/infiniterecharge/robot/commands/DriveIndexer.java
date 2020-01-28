package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsytems.Shooter;

public class DriveIndexer extends OutliersCommand {

    private Shooter _shooter;
    private OI _oi;
    public DriveIndexer(Shooter shooter, OI oi) {
        _shooter = shooter;
        _oi = oi;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double speed = _oi.getIndexerSpeed();
        _shooter.setIndexerSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
