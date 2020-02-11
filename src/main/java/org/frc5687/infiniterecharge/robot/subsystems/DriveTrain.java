package org.frc5687.infiniterecharge.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.BasicPose;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

import static org.frc5687.infiniterecharge.robot.Constants.DriveTrain.*;
import static org.frc5687.infiniterecharge.robot.util.Helpers.applySensitivityFactor;
import static org.frc5687.infiniterecharge.robot.util.Helpers.limit;

public class DriveTrain extends OutliersSubsystem {
    private CANSparkMax _leftMaster;
    private CANSparkMax _leftSlave;
    private CANSparkMax _rightMaster;
    private CANSparkMax _rightSlave;

    private CANEncoder _leftEncoder;
    private CANEncoder _rightEncoder;

    private DifferentialDriveOdometry _odometry;
    private DifferentialDriveKinematics _driveKinematics;
    private SimpleMotorFeedforward _driveFeedForward;
    private TrajectoryConfig _driveConfig;


    private OI _oi;
    private AHRS _imu;

    private Pose2d _pose;
    private Shifter _shifter;

    private double _xLength;
    private double _yLength;

    private double _oldLeftSpeedFront;
    private double _oldLeftSpeedBack;
    private double _oldRightSpeedFront;
    private double _oldRightSpeedBack;
    private boolean _isPaused = false;

    public DriveTrain(OutliersContainer container, OI oi, AHRS imu, Shifter shifter)  {
        super(container);
        _oi = oi;
        _imu = imu;
        _shifter = shifter;

        try {
            debug("Allocating motor controllers");
            _leftMaster = new CANSparkMax(RobotMap.CAN.SPARKMAX.LEFT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightMaster = new CANSparkMax(RobotMap.CAN.SPARKMAX.RIGHT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _leftSlave = new CANSparkMax(RobotMap.CAN.SPARKMAX.LEFT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightSlave = new CANSparkMax(RobotMap.CAN.SPARKMAX.RIGHT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);


            _leftEncoder = _leftMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, Constants.DriveTrain.CPR);
            _rightEncoder = _rightMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, Constants.DriveTrain.CPR);

            _leftMaster.restoreFactoryDefaults();
            _leftSlave.restoreFactoryDefaults();
            _rightMaster.restoreFactoryDefaults();
            _rightSlave.restoreFactoryDefaults();


            _leftMaster.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _leftSlave.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _rightMaster.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _rightSlave.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);

            _leftMaster.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _leftSlave.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _rightMaster.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _rightSlave.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);

            _leftMaster.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);
            _leftSlave.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);
            _rightMaster.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);
            _rightSlave.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);

            _leftMaster.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);
            _leftSlave.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);
            _rightMaster.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);
            _rightSlave.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);

            _leftMaster.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
            _leftSlave.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
            _rightMaster.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);
            _rightSlave.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);


        } catch (Exception e) {
            error("Exception allocating drive motor controllers: " + e.getMessage());
        }
        _rightEncoder.setInverted(false);
        _leftEncoder.setInverted(true);
        _leftSlave.follow(_leftMaster);
        _rightSlave.follow(_rightMaster);
        resetDriveEncoders();

        _driveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(WIDTH));
        _odometry = new DifferentialDriveOdometry(getHeading(), new Pose2d(0,0, new Rotation2d(0)));
        _driveFeedForward = new SimpleMotorFeedforward(KS_VOLTS, KV_VOLTSPR, KA_VOLTSQPR);
        _driveConfig = new TrajectoryConfig(MAX_ACCEL_MPS, MAX_ACCEL_MPS).setKinematics(_driveKinematics);

    }

    public void enableBrakeMode() {
        _leftMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _leftSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _rightMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _rightSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void disableBrakeMode() {
        _leftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _leftSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _rightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _rightSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
    public void cheesyDrive(double speed, double rotation, boolean creep, boolean override) {
        metric("Speed", speed);
        metric("Rotation", rotation);

        speed = limit(speed, Constants.DriveTrain.SPEED_LIMIT);
        Shifter.Gear gear = _shifter.getGear();

        rotation = limit(rotation, 1);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed < Constants.DriveTrain.DEADBAND && speed > -Constants.DriveTrain.DEADBAND) {
            if (!override) {
                rotation = applySensitivityFactor(rotation, _shifter.getGear() == Shifter.Gear.HIGH ? Constants.DriveTrain.ROTATION_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.ROTATION_SENSITIVITY_LOW_GEAR);
            }
            if (creep) {
                rotation = rotation * CREEP_FACTOR;
            } else {
                rotation = rotation * 0.8;
            }
            leftMotorOutput = rotation;
            rightMotorOutput = -rotation;
        } else {
            // Square the inputs (while preserving the sign) to increase fine control
            // while permitting full power.
            speed = Math.copySign(applySensitivityFactor(speed, Constants.DriveTrain.SPEED_SENSITIVITY), speed);
            if (!override) {
                rotation = applySensitivityFactor(rotation, _shifter.getGear() == Shifter.Gear.HIGH ? Constants.DriveTrain.TURNING_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.TURNING_SENSITIVITY_LOW_GEAR);
            }
            rotation = applySensitivityFactor(rotation, Constants.DriveTrain.ROTATION_SENSITIVITY);
            double delta = override ? rotation : rotation * Math.abs(speed);
            if (override) {
                // speed = Math.copySign(limit(Math.abs(speed), 1-Math.abs(delta)), speed);

                if (speed + Math.abs(delta) > 1) {
                    speed = 1 - Math.abs(delta);
                }

                if (speed - Math.abs(delta) < -1) {
                    speed = -1 + Math.abs(delta);
                }
            }
            leftMotorOutput = speed + delta;
            rightMotorOutput = speed - delta;
            metric("Str/LeftMotor", leftMotorOutput);
            metric("Str/RightMotor", rightMotorOutput);
        }

        setPower(limit(leftMotorOutput), limit(rightMotorOutput), true);
    }
    public void setPower(double leftSpeed, double rightSpeed, boolean override) {
        _leftMaster.set(leftSpeed);
        _rightMaster.set(rightSpeed);
        _leftSlave.set(leftSpeed);
        _rightSlave.set(rightSpeed);
        metric("Power/Right", rightSpeed);
        metric("Power/Left", leftSpeed);
    }
    public double getRawLeftEncoder() {
        return _leftEncoder.getPosition();
    }
    public double getRawRightEncoder() {
        return _rightEncoder.getPosition();
    }
    public double getLeftDistance() {
        return getRawLeftEncoder() * Constants.DriveTrain.ENCODER_CONVERSION;
    }
    public double getRightDistance() {
        return getRawRightEncoder() * Constants.DriveTrain.ENCODER_CONVERSION;
    }
    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }
    public double getLeftVelocity() {
        return _leftEncoder.getVelocity() * 2 * Math.PI * Units.inchesToMeters(2) / 60; //Meters Per Sec
    }
    public double getRightVelocity() {
        return _rightEncoder.getVelocity() * 2 * Math.PI * Units.inchesToMeters(2.0) / 60 ; //Meters Per Sec
    }
    public void pauseMotors() {
        _oldLeftSpeedFront = _leftMaster.get();
        _oldLeftSpeedBack = _leftSlave.get();
        _oldRightSpeedFront = _rightMaster.get();
        _oldRightSpeedBack = _rightSlave.get();
        _leftSlave.set(0);
        _leftMaster.set(0);
        _rightMaster.set(0);
        _rightSlave.set(0);
        _isPaused = true;
    }

    public void resumeMotors() {
        _leftMaster.set(_oldLeftSpeedFront);
        _leftSlave.set(_oldLeftSpeedBack);
        _rightMaster.set(_oldRightSpeedFront);
        _rightSlave.set(_oldRightSpeedBack);
        _isPaused = false;
    }

    @Override
    public void periodic() {
        _pose = _odometry.update(getHeading(), Units.inchesToMeters(getLeftDistance()), Units.inchesToMeters(getRightDistance()));
    }

    @Override
    public void updateDashboard() {

//        SmartDashboard.putBoolean("MetricTracker/Drive", true);
//        metric("X", getPose().getTranslation().getX());
//        metric("Y", getPose().getTranslation().getY());
//        metric("Heading", getPose().getRotation().getDegrees());
        metric("Distance/Left", getLeftDistance());
        metric("Distance/Right", getRightDistance());
        metric("Distance/RawLeft", getRawLeftEncoder());
        metric("Distance/RawRight", getRawRightEncoder());
        metric("Heading", getPose().getRotation().getDegrees());
    }

    public DifferentialDriveKinematics getKinematics() {
        return _driveKinematics;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-_imu.getYaw());
    }

    public double getYaw() {return _imu.getYaw();}

    public Pose2d getPose() {
        return _odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public SimpleMotorFeedforward getDriveTrainFeedForward() {
        return _driveFeedForward;
    }
    public TrajectoryConfig getDriveConfig(boolean reversed) {
        return _driveConfig.setReversed(reversed);
    }

    public void resetOdometry(Pose2d pose) {
        resetDriveEncoders();
        _odometry.resetPosition(pose, getHeading());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        _leftMaster.set(leftVolts/12);
        _rightMaster.set(rightVolts/12);
    }

    public void resetDriveEncoders() {
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
    }

    public double distanceToTarget() {
        double x = _pose.getTranslation().getX();
        double y = _pose.getTranslation().getY();
        double targetX = Constants.AutoPositions.TARGET_POSE.getTranslation().getX();
        double targetY = Constants.AutoPositions.TARGET_POSE.getTranslation().getY();
        metric("yTar", _yLength);
        metric("xTar", _xLength);
        metric("x",x);
        metric("y", y);
        _xLength = targetX - x;
        _yLength = targetY - y;
        return Math.sqrt((_xLength * _xLength) + (_yLength * _yLength));
    }

    public double getAngleToTarget() {
        double angle = 0;
        if (_yLength > 0) {
            angle = (90 - Math.toDegrees(Math.asin(_xLength / distanceToTarget())) - getHeading().getDegrees());
        } else if (_yLength < 0){
            angle =  (Math.toDegrees(Math.asin(_xLength / distanceToTarget())) - 90) - getHeading().getDegrees();
        }
        return angle;
    }

    public BasicPose getDrivePose() {
        return new BasicPose(_imu.getAngle(), _leftEncoder.getPosition(), _rightEncoder.getPosition(), 0);
    }
}
