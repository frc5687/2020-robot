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
    private boolean _allowTrim;

    public AutoTurretSetpoint(Turret turret, double angle, boolean allowTrim) {
        _turret = turret;
        _angle = angle;
        _allowTrim = allowTrim;
        addRequirements(_turret);
    }

    @Override
    public void initialize() {
        _turret.setControlMode(Turret.Control.MotionMagic);
        _turret.setMotionMagicSetpoint(_angle, _allowTrim);
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
