package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Indexer;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;

public class Spit extends OutliersCommand {
    private Shooter _shooter;
    private Indexer _indexer;
    private Turret _turret;
    private OI _oi;

    private State _state;

    private Long _endTime;
    private double _priorShooterSpeed = 0;
    private double _priorTurretAngle = 0;

    public Spit(Shooter shooter, Indexer indexer, Turret turret, OI oi) {
        _shooter = shooter;
        _indexer = indexer;
        _turret = turret;
        _oi = oi;
        addRequirements(_indexer, _shooter, _turret);
    }

    @Override
    public void initialize() {
        _endTime = null;
        _shooter.setShooting(true);
        _state = State.Start;
        _priorShooterSpeed = _shooter.getVelocitySetpoint();
        _priorTurretAngle = _turret.getPositionDegrees();
        _shooter.setShooterSpeed(Constants.Shooter.SPIT_SPEED);
        _turret.setMotionMagicSetpoint(Constants.Turret.SPIT_ANGLE);
    }

    @Override
    public void execute() {
        super.execute();
        switch (_state) {
            case Start:
                if (_shooter.isAtTargetVelocity() && _turret.isAtSetpoint()) {
                    _state = State.Spit;
                    _endTime = System.currentTimeMillis() + Constants.Shooter.TIMEOUT;
                    _indexer.setIndexerSpeed(Constants.Indexer.ADVANCE_SPEED);
                }
                break;
            case Spit:
                if (System.currentTimeMillis() > _endTime) {
                    _indexer.setIndexerSpeed(0);
                    _state = State.Resume;
                    _shooter.setShooterSpeed(_priorShooterSpeed);
                    _turret.setMotionMagicSetpoint(_priorTurretAngle);
                    _endTime = System.currentTimeMillis() + Constants.Shooter.SPIT_TIMEOUT;
                } else {
                    _indexer.setIndexerSpeed(Constants.Indexer.ADVANCE_SPEED);
                }
                break;
            case Resume:
                if (System.currentTimeMillis() > _endTime || (_shooter.isAtTargetVelocity() && _turret.isAtSetpoint())) {
                    _state = State.Done;
                }
        }
    }


    @Override
    public boolean isFinished() {
        return _state == State.Done;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _shooter.setShooting(false);
    }

    private enum State {
        Start,
        Spit,
        Resume,
        Done
    }
}
