package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Indexer;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;

public class AutoShoot extends OutliersCommand {

    private Shooter _shooter;
    private Indexer _indexer;
    private Turret _turret;


    private long _delayMillis;
    private long _endMillis = 0;

    public AutoShoot(Indexer indexer) {
        _indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        super.initialize();
        _delayMillis = System.currentTimeMillis() + Constants.Auto.AUTO_SHOOT_DELAY;
    }

    @Override
    public void execute() {
        super.execute();

        if (System.currentTimeMillis() > _delayMillis) {
            if (_endMillis==0) {
                _indexer.setIndexerSpeed(.75);
                _endMillis = System.currentTimeMillis() + Constants.Auto.AUTO_SHOOT_RUNON;
            }
        }

    }

    @Override
    public boolean isFinished() {
        return _endMillis > 0 &&  System.currentTimeMillis() > _endMillis;
    }

    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
