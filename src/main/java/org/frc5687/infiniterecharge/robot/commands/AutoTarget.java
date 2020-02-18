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
        _filter = new MedianFilter(5);
        _oi = oi;
        _speed = speed;
        _angle = angle;
        addRequirements(_turret, _shooter, _hood);
    }

    @Override
    public void initialize() {
        super.initialize();
        error("Starting AutoTarget");
        _turret.setControlMode(Turret.Control.MotionMagic);
        _limelight.enableLEDs();
        _filter.reset();
        _hood.setPosition(_angle);
        _shooter.setVelocitySpeed(_speed);

        _lights.setTargeting(true);
    }

    @Override
    public void execute() {
//        metric("Hood Setpoint", _hood.getHoodDesiredAngle(400));
//        metric("Shooter Setpoint", _shooter.getDistanceSetpoint(400));
        _hood.setPipeline();
//        if (_oi!=null) {
//            _hood.setSpeed(_oi.getHoodSpeed());
//        }
        metric("Filter ANgle", getTargetAngle());
        if (!_shooter.isShooting()) {
            _turret.setMotionMagicSetpoint(getTargetAngle());
        }
        error("Setpoint is " + (getTargetAngle()));
        if (!_shooter.isShooting()) {
            _turret.setMotionMagicSetpoint(getTargetAngle() + _turret.getManualOffset());
            error("Setpoint is " + getTargetAngle());
        }

        _lights.setReadyToshoot(_shooter.isAtTargetVelocity() && _turret.isTargetInTolerance());

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
        double targetAngle = limelightAngle + (turretAngle + angleCompensation);
        return _filter.calculate(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _lights.setTargeting(false);
        _lights.setReadyToshoot(false);
        error("Ending AutoTarget");
        _hood.setPosition(Constants.Hood.MIN_DEGREES);
//        _shooter.setShooterSpeed(Constants.Shooter.IDLE_SHOOTER_SPEED_PERCENT);
        _limelight.disableLEDs();
    }
}
