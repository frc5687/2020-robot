package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Indexer;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;

public class AutoShoot extends Shoot {

    private long _delayMillis;
    private long _endMillis = 0;

    public AutoShoot(Shooter shooter, Indexer indexer, Turret turret, OI oi, double speed) {
        super(shooter, indexer, turret, oi, speed);
    }

    @Override
    public void initialize() {
        _delayMillis = System.currentTimeMillis() + Constants.Auto.AUTO_SHOOT_DELAY;
    }

    @Override
    public void execute() {

        if (System.currentTimeMillis() > _delayMillis) {
            if (_endMillis==0) {
                super.initialize();
                _endMillis = System.currentTimeMillis() + Constants.Auto.AUTO_SHOOT_RUNON;
            }
            super.execute();
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
