package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class DeploySpinner extends OutliersCommand {

    private Spinner _spinner;

    public DeploySpinner(Spinner spinner) {
        _spinner = spinner;
    }

    @Override
    public void initialize() {
        super.initialize();
        _spinner.deploy();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
