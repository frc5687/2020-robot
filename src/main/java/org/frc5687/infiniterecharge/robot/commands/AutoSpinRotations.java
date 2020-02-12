package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class AutoSpinRotations extends OutliersCommand {
    private Spinner _spinner;
    private OI _oi;
    private int _wedgeCount = 0;
    private Spinner.Color _previousColor;

    public AutoSpinRotations(Spinner spinner, OI oi) {
        _spinner = spinner;
        _oi = oi;
        addRequirements(_spinner);
    }

    @Override
    public void initialize() {
        super.initialize();
        _spinner.resetWedgeCount();
        _spinner.spin();
    }

    @Override
    public void execute() {
        if (_spinner.getWedgeCount() > Constants.Spinner.AUTOSPIN_SLOW_AT_WEDGES) {
            _spinner.slow();
        }
    }

    @Override
    public boolean isFinished() {
        return _spinner.getWedgeCount() > Constants.Spinner.AUTOSPIN_STOP_AT_WEDGES;
    }

    @Override
    public void end(boolean interrupted) {
        _spinner.stop();
        super.end(interrupted);
    }
}
