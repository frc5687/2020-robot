package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class StowSpinner extends OutliersCommand {

    private Spinner _spinner;

    public StowSpinner(Spinner spinner) {
        _spinner = spinner;
    }

    @Override
    public void initialize() {
        super.initialize();
        _spinner.stow();
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
