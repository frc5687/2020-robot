package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;
import org.frc5687.infiniterecharge.robot.util.Limelight;

public class AutoTurretTracking extends OutliersCommand {

    private Turret _turret;
    private DriveTrain _driveTrain;
    private Limelight _limelight;
    private OI _oi;

    public AutoTurretTracking(Turret turret, DriveTrain driveTrain, Limelight limelight, OI oi) {
        _turret = turret;
        _driveTrain = driveTrain;
        _limelight = limelight;
        _oi = oi;
        addRequirements(_turret);
    }

    @Override
    public void initialize() {
//        _limelight.enableLEDs();
        _turret.setControlMode(Turret.Control.MotionMagic);
    }

    @Override
    public void execute() {
//        double position = (_limelight.getHorizontalAngle() + _turret.getPositionDegrees()) /Constants.Turret.TICKS_TO_DEGREES;
        double position = _driveTrain.getAngleToTarget() / Constants.Turret.TICKS_TO_DEGREES;

       _turret.setMotionMagicSpeed(position);
    }

    @Override
    public boolean isFinished() {
        return _oi.isKillAllPressed();
    }
}
