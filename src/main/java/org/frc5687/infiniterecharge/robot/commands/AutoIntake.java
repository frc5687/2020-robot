package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Intake;
import org.frc5687.infiniterecharge.robot.subsystems.Lights;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class AutoIntake extends OutliersCommand {
    private Intake _intake;
    private Lights _lights;

    public AutoIntake(Intake intake, Lights lights) {
        _intake = intake;
        _lights = lights;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        _intake.lowerIntake();
        _lights.setAutoIntaking(true);
    }

    @Override
    public void execute() {
        _intake.setSpeed(Constants.Intake.INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _intake.raiseIntake();
        _lights.setAutoIntaking(false);
    }
}