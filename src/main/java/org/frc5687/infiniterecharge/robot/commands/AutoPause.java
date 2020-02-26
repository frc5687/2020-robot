package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoPause extends OutliersCommand {

    private long _millis;
    private long _endMillis;

    public AutoPause(long millis, Subsystem... requirements) {
        super();
        addRequirements(requirements);
        _millis =  millis;
    }

    @Override
    public void initialize() {
        super.initialize();
        _endMillis = System.currentTimeMillis() + _millis;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > _endMillis;
    }
}
