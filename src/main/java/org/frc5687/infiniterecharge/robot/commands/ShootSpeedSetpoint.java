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
        _speed = speed;
        addRequirements(_shooter);
        logMetrics("Velocity");
    }
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MetricTracker/ShootSpeedSetpoint", true);
        super.initialize(); }

    @Override
    public void execute() {
         _shooter.setShooterSpeed(_speed);
         metric("Velocity", _shooter.getVelocity());
    }

    @Override
    public boolean isFinished() {
        return _oi.isKillAllPressed();
    }

}
