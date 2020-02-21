package org.frc5687.infiniterecharge.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotPose;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Intake;
import org.frc5687.infiniterecharge.robot.util.BasicPose;
import org.frc5687.infiniterecharge.robot.util.Helpers;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;
    private AHRS _imu;
    private PIDController _angleController;
    private Intake _intake;
    private Limelight _driveLimelight;
    private PoseTracker _poseTracker;

    private DriveState _driveState = DriveState.normal;
    private long _seekMax;
    private double _stickyLimit;
    private boolean _lockout = false;
    private int garbageCount = 0;
    private double _mediumZone;
    private double _slowZone;
    private double _slowSpeed;
    private double _mediumSpeed;
    private boolean _targetSighted;
    private double _turnSpeed;
    private long _lockEnd;
    private double _anglePIDOut;
    private boolean _useAnglePID;


    public Drive(DriveTrain driveTrain, OI oi, Intake intake, Limelight driveLimelight, PoseTracker poseTracker, AHRS imu) {
        _driveTrain = driveTrain;
        _oi = oi;
        _imu = imu;
        _intake = intake;
        _driveLimelight = driveLimelight;
        _poseTracker = poseTracker;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _mediumZone = Constants.DriveTrain.MEDIUM_ZONE_COMP;
        _slowZone = Constants.DriveTrain.SLOW_ZONE_COMP;
        _mediumSpeed =Constants.DriveTrain.MEDIUM_SPEED_COMP;
        _slowSpeed = Constants.DriveTrain.SLOW_SPEED_COMP;
//        _angleController = new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD);
//        _angleController.enableContinuousInput(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
//        _angleController.setTolerance(Constants.DriveTrain.ANGLE_TOLERANCE);
//        _useAnglePID = false;
    }

    @Override
    public void execute() {
        super.execute();
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();
//        if (wheelRotation==0 && stickSpeed != 0) {
//            _useAnglePID = true;
//            double yaw = _imu.getYaw();
//            _anglePIDOut = _angleController.calculate(yaw);
//        } else {
//            _useAnglePID = false;
//        }
//
//        if (wheelRotation==0 && _useAnglePID) {
//            metric("PID/AngleOut", _anglePIDOut);
//            metric("PID/Yaw", _imu.getYaw());
//            _driveTrain.cheesyDrive(stickSpeed, stickSpeed==0 ?  0 :_anglePIDOut, false, true);
//        } else {
            _driveTrain.cheesyDrive(stickSpeed, -wheelRotation, false, false);
//        }

        _targetSighted = _driveLimelight.isTargetSighted();
        if (!_oi.isAutoTargetDrivePressed()) {
            _stickyLimit = 1.0;
            _lockout = false;
            if (_driveState!=DriveState.normal) {
                _driveLimelight.disableLEDs();
                _driveState = DriveState.normal;
            }
        } else {
            switch (_driveState) {
                case normal:
                    // Start seeking
                    if (_intake.isLowered()) {
                        if (_intake.isIntaking()) {
                            error("Cargo intaking");
                            _driveLimelight.setPipeline(Limelight.Pipeline.PowerCell);
                            metric("Pipeline", Limelight.Pipeline.PowerCell.name());
                            _driveLimelight.disableLEDs();
                            _driveState = DriveState.seekingcells;
                        }
                    } else {
                        error("Using Limelight");
                        _driveLimelight.setPipeline(Limelight.Pipeline.Wide);
                        metric("Pipeline", Limelight.Pipeline.Wide.name());
                        _driveLimelight.enableLEDs();
                        _driveState = DriveState.seeking;
                        _seekMax = System.currentTimeMillis() + Constants.DriveTrain.SEEK_TIME;
                    }
                    break;
                case seeking:
                    if (_driveLimelight.isTargetSighted()) {
                        _turnSpeed = getTurnSpeed();
                        _lockEnd = System.currentTimeMillis() + Constants.DriveTrain.LOCK_TIME;
                        _driveState = DriveState.locking;
                    }
                    break;
                case seekingcells:
                    if (_driveLimelight.isTargetSighted()) {
                        _turnSpeed = getTurnSpeed();
                        if (_driveLimelight.isTargetCentered()) {
                            _driveLimelight.setPipeline(Limelight.Pipeline.PowerCell);
                            metric("Pipeline", Limelight.Pipeline.PowerCell.name());
                            _driveState = DriveState.trackingcells;
                        }
                    }
                    break;
                case locking:
                    if (System.currentTimeMillis() > _lockEnd || _driveLimelight.isTargetSighted()) {
                        // Note that we could also wait until the target is centered to lock...which might make more sense.
                        // Just add  && _limelight.isTargetCentered() to the condition above
                        _driveLimelight.setPipeline(Limelight.Pipeline.Wide);
                        metric("Pipeline", Limelight.Pipeline.Wide.name());
                        _driveState = DriveState.tracking;
                    }
                    _turnSpeed = getTurnSpeed();
                    break;
                case tracking:
                    _turnSpeed = getTurnSpeed();
                    break;
                case trackingcells:
                    _turnSpeed = getTurnSpeed();
                    break;
            }
        }
        stickSpeed = limitSpeed(stickSpeed);
        if (_driveState == DriveState.normal) {
            _driveTrain.cheesyDrive(stickSpeed, wheelRotation, _oi.isCreepPressed(), false);
        } else {
            _driveTrain.cheesyDrive(stickSpeed, _turnSpeed, false, true);
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    protected double getTurnSpeed() {
        metric("Lockout", _lockout);
        if (_lockout || (!_driveLimelight.isTargetSighted())) { return 0; }
        double distance = _driveLimelight.getTargetDistance();

        _seekMax = System.currentTimeMillis() + Constants.DriveTrain.DROPOUT_TIME;

        if (distance > 0 && distance < Constants.Auto.Drive.MIN_TRACK_DISTANCE) {
            // We're too close to trust the target!
            error("Target too close at " + distance + ", count=" + garbageCount);
            garbageCount++;
            if (garbageCount > Constants.Auto.Drive.MAX_GARBAGE) {
                _lockout = true;
                error("Garbagecount=" + garbageCount + " setting lockout.");
            }
            return 0;
        }
        garbageCount = 0;
        double limelightAngle = _driveLimelight.getHorizontalAngle();
        double yaw = _driveTrain.getHeading().getDegrees();

        // Find the pose of the robot _when the picture was taken_
        long timeKey = System.currentTimeMillis() - (long)_driveLimelight.getLatency();
        RobotPose pose = (RobotPose)_poseTracker.get(timeKey);

        // Get the angle from the pose if one was found--otherwise use yaw
        double poseAngle = pose == null ? -yaw : -pose.getDrivePose().getAngle();

        // Now adjust the limelight angle based on the change in yaw from when the picture was taken to now
        double offsetCompensation = -yaw - poseAngle;
        double targetAngle = limelightAngle - offsetCompensation;

        if (distance > 0 && distance < Constants.Auto.Drive.MIN_TRACK_DISTANCE) {
            targetAngle *= (distance / 48);
        }

        metric("Pose", pose==null?0:pose.getMillis());
        metric("Yaw", yaw);
        metric("PoseAngle", poseAngle);
        metric("LimelightAngle", limelightAngle);
        metric("TargetAngle", targetAngle);

        return targetAngle * Constants.Auto.Drive.STEER_K;
    }
    private double limitSpeed(double speed) {
        double limit = 1;
        if (_driveState!=DriveState.normal) {
            if(_driveLimelight.isTargetSighted()) {
                _seekMax = System.currentTimeMillis() + Constants.DriveTrain.DROPOUT_TIME;
                double distance = _driveLimelight.getTargetDistance();
                metric("TargetDistance", distance);
                if (distance  > 0) {
                    if (distance < _mediumZone) {
                        limit = _mediumSpeed;
                        _stickyLimit = limit;
                    }
                    if (distance < _slowZone) {
                        limit = _slowSpeed;
                        _stickyLimit = limit;
                    }
                }
            } else if (System.currentTimeMillis() > _seekMax){
                metric("TargetDistance", -999);
                // We've been seeking for more than the max allowed...slow the robot down!
                _oi.pulseDriver(1);
            }
        }
        limit = Math.min(limit, _stickyLimit);
        if (_driveLimelight.isTargetSighted()) {
            error("Limiting speed for middle hatch");
            // TODO: Ben says this limit is bad with limelight obstructions limit = Math.min(0.5, limit);
        }
        double limited = Helpers.limit(speed, -limit, limit);
        metric("limit", limit);
        metric("limited", limited);
        return limited;
    }

    public enum DriveState {
        normal(0),
        seeking(1),
        locking(2),
        tracking(3),
        seekingcells(4),
        trackingcells(5),
        lost(6);

        private int _value;

        DriveState(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }

}
