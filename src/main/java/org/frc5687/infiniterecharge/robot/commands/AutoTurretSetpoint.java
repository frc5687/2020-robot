package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;
import org.frc5687.infiniterecharge.robot.util.Limelight;

public class AutoTurretSetpoint extends OutliersCommand {
    private Turret _turret;
    private DriveTrain _driveTrain;
    private Limelight _limelight;
    private OI _oi;
    private double _angle;

    public AutoTurretSetpoint(Turret turret, double angle) {
        _turret = turret;
        _angle = angle;
        addRequirements(_turret);
    }

    @Override
    public void initialize() {
        _turret.setControlMode(Turret.Control.MotionMagic);
        _turret.setMotionMagicSetpoint(_angle);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        error("At setpoint");
        return _turret.isAtSetpoint();
    }

}
