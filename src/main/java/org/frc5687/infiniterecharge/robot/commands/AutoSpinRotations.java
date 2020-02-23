package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Skywalker;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class AutoSpinRotations extends OutliersCommand {
    private Spinner _spinner;
    private OI _oi;
    private int _wedgeCount = 0;
    private Spinner.MatchedColor _previousMatchedColor;

    public AutoSpinRotations(Spinner spinner, OI oi, Skywalker skywalker) {
        _spinner = spinner;
        _oi = oi;
        addRequirements(_spinner, skywalker);
    }

    @Override
    public void initialize() {
        super.initialize();
        _spinner.resetWedgeCount();
        _spinner.deploy();
        _spinner.spin();
    }

    @Override
    public void execute() {
        if (_spinner.getWedgeCount() > Constants.Spinner.AUTOSPIN_SLOW_AT_WEDGES) {
            _spinner.spinSlowly();
        }
    }

    @Override
    public boolean isFinished() {
        return _spinner.getWedgeCount() > Constants.Spinner.AUTOSPIN_STOP_AT_WEDGES;
    }

    @Override
    public void end(boolean interrupted) {
        _spinner.stop();
        _spinner.stow();
        super.end(interrupted);
    }
}
