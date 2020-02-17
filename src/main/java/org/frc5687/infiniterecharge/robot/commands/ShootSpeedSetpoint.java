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
        logMetrics("VelocityRPM");
    }
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MetricTracker/ShootSpeedSetpoint", true);
        super.initialize();
        _shooter.setVelocitySpeed(_speed);
    }

    @Override
    public void execute() {
//         _shooter.setShooterSpeed(_speed);
         metric("VelocityRPM", _shooter.getRPM());
    }

    @Override
    public boolean isFinished() {
        return _oi.isKillAllPressed();
    }

}
