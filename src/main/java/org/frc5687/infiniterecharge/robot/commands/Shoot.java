package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Indexer;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;

public class Shoot extends OutliersCommand {
    private Shooter _shooter;
    private Indexer _indexer;
    private Turret _turret;
    private OI _oi;

    private double _speed;

    private Long _endTime;
    public Shoot(Shooter shooter, Indexer indexer, Turret turret, OI oi) {
        _shooter = shooter;
        _indexer = indexer;
        _turret = turret;
        _oi = oi;
        addRequirements(_indexer);
        logMetrics("Velocity");
    }

    @Override
    public void initialize() {
        _endTime = null;
        SmartDashboard.putBoolean("MetricTracker/Shoot", true);
        _shooter.setShooting(true);
    }

    @Override
    public void execute() {
        super.execute();

        if ((_turret.isTargetInTolerance() && _shooter.isAtTargetVelocity()) || _oi.isOverridePressed()) {
            error("SHOOTING AT VELOCITY " + _shooter.getRPM());
            error("INDEXING NOW");
            _endTime = System.currentTimeMillis() + Constants.Shooter.TIMEOUT;
            _indexer.setIndexerSpeed(Constants.Indexer.ADVANCE_SPEED);
        }
    }


    @Override
    public boolean isFinished() {
        if (_endTime == null) {
            return false;
        }
        return System.currentTimeMillis() >= _endTime;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _shooter.setShooting(false);
    }
}
