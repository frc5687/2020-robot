package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotPose;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;

public class AutoTarget extends OutliersCommand {

    private Turret _turret;
    private Shooter _shooter;
    private Hood _hood;
    private Limelight _limelight;
    private PoseTracker _poseTracker;

    public AutoTarget(Turret turret, Shooter shooter, Hood hood, Limelight limelight, PoseTracker poseTracker) {
        _turret = turret;
        _shooter = shooter;
        _hood = hood;
        _limelight = limelight;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _turret.setMotionMagicSpeed(getTargetAngle()/Constants.Turret.TICKS_TO_DEGREES);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    protected double getTargetAngle() {
        double limelightAngle = _limelight.getHorizontalAngle();
        double turretAngle = _turret.getPositionDegrees();

        long timekey = System.currentTimeMillis() - (long)_limelight.getLatency();
        RobotPose pose = (RobotPose)_poseTracker.get(timekey);

        double poseAngle = pose == null ? turretAngle : pose.getTurretPose().getAngle();
        double angleCompensation = turretAngle - poseAngle;
        double targetAngle = angleCompensation + limelightAngle;
        return targetAngle;
    }

}
