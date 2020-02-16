package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;
import org.frc5687.infiniterecharge.robot.subsystems.Skywalker;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class AutoBalance extends OutliersCommand {
    private Skywalker _skywalker;
    private Spinner _spinner;
    private OI _oi;

    public AutoBalance(Skywalker skywalker, Spinner spinner, OI oi){
        _skywalker = skywalker;
        _spinner = spinner;
        _oi = oi;
        addRequirements(_skywalker, _spinner);
    }

    @Override
    public void initialize() {
        super.initialize();

    }

    @Override
    public void execute() {
        // State machine logic below:
        super.execute();
        double speed = 0;
        if (_oi.isOverridePressed()) {
            metric("Skywalker state", "Overridden");
            speed = _oi.getSkywalkerSpeed();
        } else if (_skywalker.isUpTriggered()) {
            metric("Skywalker State", "Up Triggered, Ascending");
            speed = Constants.Skywalker.UPSPEED;
        } else if (_skywalker.isDownTriggered()) {
            metric("Skywalker State", "Down Triggered, Descending");
            speed = Constants.Skywalker.DOWNSPEED;
        } else {
            metric("Skywalker State", "Level");
            speed = Constants.Skywalker.SKYWALKER_TENSION_SPEED;
        }
        metric("Skywalker speed", speed);
        _spinner.setSpeed(speed);
    }
}
