package org.frc5687.infiniterecharge.robot.commands;


import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class KillAll extends OutliersCommand {
    private boolean _finished;

    private DriveTrain _driveTrain;
    private Shooter _shooter;

    public KillAll(DriveTrain driveTrain, Shooter shooter) {
        _driveTrain = driveTrain;
        _shooter = shooter;
        addRequirements(driveTrain, shooter);
    }

    @Override
    public void initialize() {
        _finished = true;
        _driveTrain.enableBrakeMode();
        _shooter.setShooterSpeed(0);
        _shooter.setIndexerSpeed(0);
        error("Initialize KillAll Command");
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
