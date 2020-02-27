package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class ShootSpeedSetpoint extends OutliersCommand {

    private Shooter _shooter;
    private OI _oi;
    private double _speed;

    public ShootSpeedSetpoint(Shooter shooter, OI oi, double speed) {
        _shooter = shooter;
        _oi = oi;
        _speed = speed; // in RPM
        addRequirements(_shooter);
    }
    @Override
    public void initialize() {
        super.initialize();
        _shooter.setVelocitySpeed(_speed, true);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return _oi.isKillAllPressed();
    }

}
