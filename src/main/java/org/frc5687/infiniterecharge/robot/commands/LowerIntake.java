package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Intake;

public class LowerIntake extends OutliersCommand {
    private Intake _intake;

    public LowerIntake(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        _intake.lowerIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
