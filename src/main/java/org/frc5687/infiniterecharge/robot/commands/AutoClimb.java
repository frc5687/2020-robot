package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Climber;
import org.frc5687.infiniterecharge.robot.subsystems.Skywalker;
import org.frc5687.infiniterecharge.robot.Constants;

public class AutoClimb extends OutliersCommand {
    private Climber _climber;
    private Skywalker _skywalker;

    public AutoClimb (Climber climber, Skywalker skywalker){
        _climber = climber;
        _skywalker = skywalker;

        addRequirements(_climber ,_skywalker);
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
        if (_skywalker.isUpTriggered()) {
            metric("Skywalker State", "Up Triggered, Ascending");
            speed = Constants.Skywalker.UPSPEED;
        } else if (_skywalker.isDownTriggered()) {
            metric("Skywalker State", "Down Triggered, Descending");
            speed = Constants.Skywalker.DOWNSPEED;
        } else if (!_skywalker.isDownTriggered() && !_skywalker.isUpTriggered()) {
            metric("Skywalker State", "Level");
            speed = Constants.Skywalker.SKYWALKER_TENSION_SPEED;
        } else {
            metric("Skywalker State", "Broken, Abort Climb");
            speed = Constants.Skywalker.SKYWALKER_TENSION_SPEED;
        }

        _skywalker.setSpeed(speed);

    }
}
