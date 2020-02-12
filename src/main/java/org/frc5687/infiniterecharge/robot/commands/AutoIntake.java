package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Intake;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class AutoIntake extends OutliersCommand {
    private Intake _intake;
    private Spinner _spinner;

    public AutoIntake(Intake intake, Spinner spinner) {
        _intake = intake;
        _spinner = spinner;
        addRequirements(_intake, _spinner);
    }

    @Override
    public void initialize() {
        super.initialize();
        _intake.lowerIntake();
    }

    @Override
    public void execute() {
        _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
        _spinner.setSpeed(Constants.Spinner.PRE_INDEXER_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.raiseIntake();
    }
}