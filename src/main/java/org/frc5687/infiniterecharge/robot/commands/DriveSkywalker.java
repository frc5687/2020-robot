package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Skywalker;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class DriveSkywalker extends OutliersCommand{
    private Skywalker _skywalker;
    private Spinner _spinner;
    private OI _oi;

    @Override
    public void initialize() {
        super.initialize();
    }

    public DriveSkywalker(Skywalker skywalker, Spinner spinner, OI oi) {
        _skywalker = skywalker;
        _spinner = spinner;
        _oi = oi;
        addRequirements(_skywalker, _spinner);
    }

    @Override
    public void execute() {
        error("Speed is:" + _oi.getSkywalkerSpeed());
        _spinner.setSpeed(_oi.getSkywalkerSpeed());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
