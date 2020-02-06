package org.frc5687.infiniterecharge.robot.commands;

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

    private double _setpoint;
    private Long _endTime;
    private long _startTime;
    public Shoot(Shooter shooter, Indexer indexer, Turret turret, OI oi) {
        _shooter = shooter;
        _indexer = indexer;
        _turret = turret;
        _oi = oi;
        addRequirements(_shooter, _indexer);
    }

    @Override
    public void initialize() {
        _startTime = System.currentTimeMillis();
        _endTime = null;
    }

    @Override
    public void execute() {
        super.execute();
        _setpoint = _shooter.getDistanceSetpoint();
        _shooter.setShooterSpeed(_setpoint);
        if (_turret.isTargetInTolerance() || _oi.isOverridePressed()) {
            if (_shooter.isAtVelocity(_setpoint) || _oi.isOverridePressed()) {
                _indexer.setIndexerSpeed(Constants.Indexer.ADVANCE_SPEED);
                _endTime = _startTime + Constants.Shooter.TIMEOUT;
            }
        }
    }


    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() >= _endTime);
    }

}
