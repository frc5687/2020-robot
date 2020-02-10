package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Skywalker;

public class DriveSkywalker extends OutliersCommand{
    private Skywalker _skywalker;
    private OI _oi;

    @Override
    public void initialize() {
        super.initialize();
    }

    public DriveSkywalker(Skywalker skywalker, OI oi) {
        _skywalker = skywalker;
        _oi = oi;
        addRequirements(_skywalker);
    }

    @Override
    public void execute() {

        _skywalker.setSpeed(_oi.getSkywalkerSpeed());

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
