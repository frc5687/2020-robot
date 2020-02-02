package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Indexer;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class DriveIndexer extends OutliersCommand {

    private Indexer _indexer;
    private OI _oi;
    public DriveIndexer(Indexer indexer, OI oi) {
        _indexer = indexer;
        _oi = oi;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double speed = _oi.getIndexerSpeed();
        _indexer.setIndexerSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
