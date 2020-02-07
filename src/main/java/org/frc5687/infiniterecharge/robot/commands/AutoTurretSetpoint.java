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

    public AutoTurretSetpoint(Turret turret, DriveTrain driveTrain, Limelight limelight, OI oi, double angle) {
        _turret = turret;
        _driveTrain = driveTrain;
        _limelight = limelight;
        _oi = oi;
        _angle = angle;
        addRequirements(_turret);
    }

    @Override
    public void initialize() {
//        _limelight.enableLEDs();
        _turret.setControlMode(Turret.Control.MotionMagic);
    }

    @Override
    public void execute() {
        double position = _angle / Constants.Turret.TICKS_TO_DEGREES;
        _turret.setMotionMagicSpeed(position);
    }

    @Override
    public boolean isFinished() {
        if (_oi.isKillAllPressed()) {
            error("Ending KillAll");
            return true;
        }
        error("At setpoint");
        return _turret.isAtSetpoint();
    }

}
