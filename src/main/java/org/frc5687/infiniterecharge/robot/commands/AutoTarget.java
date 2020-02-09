package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotPose;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Hood;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;

public class AutoTarget extends OutliersCommand {

    private Turret _turret;
    private Shooter _shooter;
    private Hood _hood;
    private DriveTrain _driveTrain;
    private Limelight _limelight;
    private PoseTracker _poseTracker;

    public AutoTarget(Turret turret, Shooter shooter, Hood hood, Limelight limelight, DriveTrain driveTrain, PoseTracker poseTracker) {
        _turret = turret;
        _shooter = shooter;
        _hood = hood;
        _driveTrain = driveTrain;
        _limelight = limelight;
    }

    @Override
    public void initialize() {
        super.initialize();
        _limelight.enableLEDs();
    }

    @Override
    public void execute() {
        _hood.setPosition(_hood.getHoodDesiredAngle(_driveTrain.distanceToTarget()));
        _turret.setMotionMagicSetpoint(getTargetAngle());
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

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _hood.setPosition(Constants.Hood.MIN_DEGREES);
        _limelight.disableLEDs();
    }
}
