package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.MedianFilter;
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
    private MedianFilter _filter;
    private double _speed;
    private double _angle;

    public AutoTarget(Turret turret,
                      Shooter shooter,
                      Hood hood,
                      Limelight limelight,
                      DriveTrain driveTrain,
                      PoseTracker poseTracker,
                      double angle,
                      double speed) {
        _turret = turret;
        _shooter = shooter;
        _hood = hood;
        _driveTrain = driveTrain;
        _limelight = limelight;
        _poseTracker = poseTracker;
        _filter = new MedianFilter(10);
        _angle = angle;
        _speed = speed;
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
        _shooter.setShooterSpeed(_speed);
    }

    @Override
    public void execute() {
        _turret.setMotionMagicSetpoint(_limelight.getHorizontalAngle() + _turret.getPositionDegrees());
        error("Setpoint is " + (_limelight.getHorizontalAngle() + _turret.getPositionDegrees()));
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
        double targetAngle = limelightAngle + angleCompensation;
        return _filter.calculate(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        error("Ending AutoTarget");
        _hood.setPosition(Constants.Hood.MIN_DEGREES);
        _shooter.setShooterSpeed(Constants.Shooter.IDLE_SHOOTER_SPEED_PERCENT);
        _limelight.disableLEDs();
    }
}
