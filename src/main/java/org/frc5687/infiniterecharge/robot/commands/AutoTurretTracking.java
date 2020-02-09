package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;
import org.frc5687.infiniterecharge.robot.util.TurretPose;

public class AutoTurretTracking extends OutliersCommand {

    private Turret _turret;
    private DriveTrain _driveTrain;
    private Limelight _limelight;
    private OI _oi;
    private PoseTracker _poseTracker;

    public AutoTurretTracking(Turret turret, DriveTrain driveTrain, Limelight limelight, OI oi, PoseTracker poseTracker) {
        _turret = turret;
        _driveTrain = driveTrain;
        _limelight = limelight;
        _oi = oi;
        _poseTracker = poseTracker;
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

       _turret.setMotionMagicSetpoint(position);
    }

    public double getVisionSetpoint() {
        double limelightAngle = _limelight.getHorizontalAngle();
        double turretAngle = _turret.getPositionDegrees();

        long timekey = System.currentTimeMillis() - (long)_limelight.getLatency();
        TurretPose pose = (TurretPose)_poseTracker.get(timekey);

        double poseAngle = pose == null ? turretAngle : pose.getAngle();

        double offsetCompensation = turretAngle - poseAngle;
        double targetAngle = limelightAngle - offsetCompensation;
        return targetAngle;

    }

    @Override
    public boolean isFinished() {
        return _oi.isKillAllPressed();
    }
}
