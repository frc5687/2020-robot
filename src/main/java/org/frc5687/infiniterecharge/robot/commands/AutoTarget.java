package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.MedianFilter;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotPose;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;

public class AutoTarget extends OutliersCommand {

    private Turret _turret;
    private Shooter _shooter;
    private Hood _hood;
    private DriveTrain _driveTrain;
    private Lights _lights;
    private Limelight _limelight;
    private PoseTracker _poseTracker;
    private MedianFilter _filter;
    private double _speed;
    private double _angle;
    private OI _oi;

    public AutoTarget(Turret turret,
                      Shooter shooter,
                      Hood hood,
                      Limelight limelight,
                      DriveTrain driveTrain,
                      PoseTracker poseTracker,
                      Lights lights,
                      OI oi,
                      double speed,
                      double angle) {
        _turret = turret;
        _shooter = shooter;
        _hood = hood;
        _driveTrain = driveTrain;
        _limelight = limelight;
        _lights = lights;
        _poseTracker = poseTracker;
        _filter = new MedianFilter(10);
        _oi = oi;
        _speed = speed;
        _angle = angle;
        addRequirements(_turret, _shooter, _hood);
    }

    @Override
    public void initialize() {
        super.initialize();
        _turret.setControlMode(Turret.Control.MotionMagic);
        _limelight.enableLEDs();
        _filter.reset();
        _hood.setPosition(_angle);
        _shooter.setVelocitySpeed(_speed);

        _lights.setTargeting(true);
    }

    @Override
    public void execute() {
        if (!_shooter.isShooting()) {
            _turret.setMotionMagicSetpoint(_limelight.getHorizontalAngle() + _turret.getPositionDegrees());
        }
        if (!_shooter.isShooting()) {
            _turret.setMotionMagicSetpoint(_limelight.getHorizontalAngle() + _turret.getPositionDegrees() + _turret.getManualOffset());
        }
        _lights.setReadyToshoot(_shooter.isAtTargetVelocity() && _turret.isTargetInTolerance());

    }


    @Override
    public boolean isFinished() {
        return false;
    }

    protected double getTargetAngle() {
        double limelightAngle = _filter.calculate(_limelight.getHorizontalAngle());
        double turretAngle = _turret.getPositionDegrees();

        long timekey = System.currentTimeMillis() - (long)_limelight.getLatency();
        RobotPose pose = (RobotPose)_poseTracker.get(timekey);

        double poseAngle = pose == null ? turretAngle : pose.getTurretPose().getAngle();
        double angleCompensation = turretAngle - poseAngle;
        double targetAngle = limelightAngle + (turretAngle + angleCompensation);
        return targetAngle;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _lights.setTargeting(false);
        _lights.setReadyToshoot(false);
        _hood.setPosition(Constants.Hood.MIN_DEGREES);
        _limelight.disableLEDs();
    }
}
