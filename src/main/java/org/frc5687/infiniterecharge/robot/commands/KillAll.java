package org.frc5687.infiniterecharge.robot.commands;


import org.frc5687.infiniterecharge.robot.subsystems.*;

public class KillAll extends OutliersCommand {
    private boolean _finished;

    private DriveTrain _driveTrain;
    private Shooter _shooter;
    private Indexer _indexer;
    private Intake _intake;
    private Turret _turret;
    private Hood _hood;

    public KillAll(DriveTrain driveTrain, Shooter shooter, Indexer indexer, Intake intake, Turret turret, Hood hood) {
        _driveTrain = driveTrain;
        _shooter = shooter;
        _indexer = indexer;
        _intake = intake;
        _turret = turret;
        _hood = hood;
        addRequirements(_driveTrain, _shooter, _indexer, _intake, _turret, _hood);
    }

    @Override
    public void initialize() {
        error("Initialize KillAll Command");
        _finished = true;
        _driveTrain.enableBrakeMode();
        _shooter.setShooterSpeed(0);
        _indexer.setIndexerSpeed(0);
        _intake.setSpeed(0);
        _turret.setSpeed(0);
        _hood.setSpeed(0);
    }

    @Override
    public void end(boolean interrupted)  {
        error("Ending KillAll Command");
    }

    @Override
    public boolean isFinished() {
        return _finished;
    }
}
