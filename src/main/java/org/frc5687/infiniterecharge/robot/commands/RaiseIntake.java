package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Intake;

public class RaiseIntake extends OutliersCommand {
    private Intake _intake;
    public RaiseIntake(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        _intake.raiseIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
