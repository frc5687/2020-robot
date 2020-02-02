package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class Shoot extends OutliersCommand {
    private Shooter _shooter;
    private OI _oi;
    public Shoot(Shooter shooter, OI oi) {
        _shooter = shooter;
        _oi = oi;
        addRequirements(_shooter);
    }
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        double speed = _oi.getShooterSpeed();
        _shooter.setShooterSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
