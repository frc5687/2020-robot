package frc.robot.commands;


import frc.robot.Robot;

public class KillAll extends OutliersCommand {
    private boolean _finished;
    private Robot _robot;

    public KillAll(Robot robot) {
        addRequirements(robot.getDriveTrain());
        _robot = robot;
    }

    @Override
    public void initialize() {
        _robot.getDriveTrain().enableBrakeMode();
        _finished = true;
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
