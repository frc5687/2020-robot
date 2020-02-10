package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotPose;
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
        double position = _driveTrain.getAngleToTarget();
        metric("angle target", position);
       _turret.setMotionMagicSetpoint(position);
    }

    public double getVisionSetpoint() {
        double limelightAngle = _limelight.getHorizontalAngle();
        double turretAngle = _turret.getPositionDegrees();

        long timekey = System.currentTimeMillis() - (long)_limelight.getLatency();
        RobotPose pose = (RobotPose)_poseTracker.get(timekey);

        double poseAngle = pose == null ? turretAngle : pose.getTurretPose().getAngle();

        double offsetCompensation = turretAngle - poseAngle;
        double targetAngle = limelightAngle + offsetCompensation;
        return targetAngle;
    }

    @Override
    public boolean isFinished() {
        return _oi.isKillAllPressed();
    }
}
