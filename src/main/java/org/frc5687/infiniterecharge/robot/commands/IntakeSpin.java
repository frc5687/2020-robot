package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.commands.OutliersCommand;
import org.frc5687.infiniterecharge.robot.subsystems.Intake;

public class IntakeSpin extends OutliersCommand {
    private OI _oi;
    private Intake _intake;
    private double _rotationSpeed;

    public IntakeSpin(Intake intake, OI oi) {
        _intake = intake;
        _oi = oi;
        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double intakeSpeed = _oi.getIntakeSpeed();
        _intake.setSpeed(intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
