package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Intake;

public class MidIntake extends OutliersCommand {
    private Intake _intake;
    public MidIntake(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        _intake.midIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
