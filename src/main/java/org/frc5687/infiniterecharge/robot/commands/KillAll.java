package org.frc5687.infiniterecharge.robot.commands;


import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class KillAll extends OutliersCommand {
    private boolean _finished;

    private DriveTrain _driveTrain;

    public KillAll(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        _finished = true;
        _driveTrain.enableBrakeMode();
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
