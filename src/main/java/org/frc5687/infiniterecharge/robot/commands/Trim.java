package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.ISupportsTrim;
import org.frc5687.infiniterecharge.robot.subsystems.OutliersSubsystem;

public class Trim extends OutliersCommand {
    private ISupportsTrim _subsystem;
    private double _increment;

    public Trim(ISupportsTrim subsystem, double increment) {
        _subsystem = subsystem;
        _increment = increment;
    }

    @Override
    public void initialize() {
        _subsystem.trim(_increment);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
