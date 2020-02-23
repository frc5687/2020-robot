package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.DriveTurret;
import org.frc5687.infiniterecharge.robot.util.Helpers;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

import org.frc5687.infiniterecharge.robot.Constants.AutoPositions.*;
import org.frc5687.infiniterecharge.robot.util.TurretPose;

import java.lang.annotation.Target;


public class Turret extends OutliersSubsystem {

    private TalonSRX _turretController;
    private Limelight _limelight;
    private DriveTrain _driveTrain;
    private Hood _hood;
    private OI _oi;

    private int _positionPIDSlot = 0;
    private int _velocityPIDSlot = 1;
    private int _motionMagicPIDSlot = 2;
    private double _positionABS;
    private double _position;

    private double _manualOffset = 0;

    public Turret(OutliersContainer container, DriveTrain driveTrain, Hood hood, Limelight limelight, OI oi) {
        super(container);
        _driveTrain = driveTrain;
        _hood = hood;
        _limelight = limelight;
        _oi = oi;

        try {
            debug("allocating turret motor");
            _turretController = new TalonSRX(RobotMap.CAN.TALONSRX.TURRET);
            _turretController.setInverted(Constants.Turret.INVERTED);
            _turretController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
            _turretController.setSensorPhase(Constants.Turret.SENSOR_PHASE_INVERTED);
            _turretController.configForwardSoftLimitThreshold((int)(Constants.Turret.MAX_DEGREES/Constants.Turret.TICKS_TO_DEGREES), 30);
            _turretController.configForwardSoftLimitEnable(true, 30);
            _turretController.configReverseSoftLimitThreshold((int) (Constants.Turret.MIN_DEGREES/Constants.Turret.TICKS_TO_DEGREES), 30);
            _turretController.configReverseSoftLimitEnable(true, 30);
            _turretController.configMotionCruiseVelocity(Constants.Turret.CRUISE_VELOCITY);
            _turretController.configMotionAcceleration(Constants.Turret.ACCELERATION);
            _turretController.configVoltageMeasurementFilter(8);
            _turretController.enableVoltageCompensation(true);
            _turretController.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10,20);
            _turretController.configClosedloopRamp(0,50);
            _turretController.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 30);
            _turretController.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 30);
            _turretController.setNeutralMode(NeutralMode.Brake);
        } catch (Exception e) {
            error("error allocating turret motors " + e.getMessage());
        }
        zeroSensors();
    }

    public void setSpeed(double speed) {
        _turretController.set(ControlMode.PercentOutput, speed);
    }

    public void setControlMode(Control control) {
        if (control == Control.Position) {
            _turretController.config_kP(_positionPIDSlot, Constants.Turret.Position.kP, 50);
            _turretController.config_kI(_positionPIDSlot, Constants.Turret.Position.kI, 50);
            _turretController.config_kD(_positionPIDSlot, Constants.Turret.Position.kD, 50);
            _turretController.config_kF(_positionPIDSlot, Constants.Turret.Position.kF, 50);
            _turretController.selectProfileSlot(_positionPIDSlot, 0);
        } else if (control == Control.Velocity) {
            _turretController.config_kP(_velocityPIDSlot, Constants.Turret.Velocity.kP, 50);
            _turretController.config_kI(_velocityPIDSlot, Constants.Turret.Velocity.kI, 50);
            _turretController.config_kD(_velocityPIDSlot, Constants.Turret.Velocity.kD, 50);
            _turretController.config_kF(_velocityPIDSlot, Constants.Turret.Velocity.kF, 50);
            _turretController.selectProfileSlot(_velocityPIDSlot, 0);
        } else if (control == Control.MotionMagic) {
            _turretController.config_kP(_motionMagicPIDSlot, Constants.Turret.MotionMagic.kP, 50);
            _turretController.config_kI(_motionMagicPIDSlot, Constants.Turret.MotionMagic.kI, 50);
            _turretController.config_kD(_motionMagicPIDSlot, Constants.Turret.MotionMagic.kD, 50);
            _turretController.config_kF(_motionMagicPIDSlot, Constants.Turret.MotionMagic.kF, 50);
            _turretController.configAllowableClosedloopError(_motionMagicPIDSlot, 0, 50);
            _turretController.selectProfileSlot(_motionMagicPIDSlot, 0);
        }
    }

    public void setMotionMagicSetpoint(double angle) {
        if (angle > Constants.Turret.MAX_DEGREES) {
            angle =  angle - 360;
        } else if (angle < Constants.Turret.MIN_DEGREES) {
            angle = angle + 360;
        }
        angle = Helpers.limit(angle, Constants.Turret.MIN_DEGREES, Constants.Turret.MAX_DEGREES);
        _turretController.set(ControlMode.MotionMagic, (angle/Constants.Turret.TICKS_TO_DEGREES));
    }

    public boolean isAtSetpoint() {
        return _turretController.isMotionProfileFinished();
    }

    public void enableBreakMode() {
        _turretController.setNeutralMode(NeutralMode.Brake);
    }

    public void disableBreakMode() {
        _turretController.setNeutralMode(NeutralMode.Coast);
    }

    public Pose2d updatePose() {
        Pose2d prevPose = _driveTrain.getPose();
        double distance = Units.inchesToMeters(20);
        // big dumb turret angle doesn't effect pose.
//        double alpha = (90 -(getPositionDegrees() + _limelight.getHorizontalAngle())) - _driveTrain.getHeading().getDegrees();
        double alpha = 90 - Math.abs(_limelight.getHorizontalAngle());
        double x = Math.sin(Math.toRadians(alpha)) * distance;
        double y = Math.cos(Math.toRadians(alpha)) * distance;
        double poseX = Constants.AutoPositions.TARGET_POSE.getTranslation().getX() - x;
        double poseY = 0;
        if (prevPose.getTranslation().getY() < Constants.AutoPositions.TARGET_POSE.getTranslation().getY()) {
            poseY = Constants.AutoPositions.TARGET_POSE.getTranslation().getY() - y;
        } else if (prevPose.getTranslation().getY() > Constants.AutoPositions.TARGET_POSE.getTranslation().getY()) {
            poseY = Constants.AutoPositions.TARGET_POSE.getTranslation().getY() + y;
        }
        return new Pose2d(poseX, poseY, _driveTrain.getHeading());
    }

    @Override
    public void periodic() {
//        if (_oi.isAutoTargetPressed()) {
//            _driveTrain.resetOdometry(updatePose());
//        }
        if (_driveTrain.getPose().getTranslation().getX() > 0) {
           _limelight.setPipeline(Limelight.Pipeline.TwoTimes);
        } else if (_driveTrain.getPose().getTranslation().getX() < 0) {
            _limelight.setPipeline(Limelight.Pipeline.Wide);
        }
    }

    public void updateDashboard() {
        metric("Position Degrees", getPositionDegrees());
        metric("Absolute Pos", getAbsoluteEncoderPosition());
        metric("forward", _turretController.isFwdLimitSwitchClosed());
        metric("rev", _turretController.isRevLimitSwitchClosed());
    }

    public void zeroSensors() {
        _position = _positionABS;
        _position = _position/Constants.Turret.TICKS_TO_DEGREES;
        _turretController.setSelectedSensorPosition((int) _position);
    }

    public void adjustOffset(double amount) {
        _manualOffset += amount;
    }

    public int getPositionTicks() {
        return _turretController.getSelectedSensorPosition(0);
    }

    public double getPositionDegrees() {
        return getPositionTicks() * Constants.Turret.TICKS_TO_DEGREES;
    }

    public int getVelocityTicksPer100ms() {
        return _turretController.getSelectedSensorVelocity(0);
    }

    //taken from 254
    public int getAbsoluteEncoderRawPosition() {
        int rawABS = _turretController.getSensorCollection().getPulseWidthPosition();
        int rawRollOver = rawABS % 4096;
        return rawRollOver + (rawRollOver < 0 ? rawRollOver + 4096 : 0);
    }
    public double getAbsoluteEncoderPosition() {
        _positionABS = (getAbsoluteEncoderRawPosition() * Constants.Turret.TICKS_TO_DEGREES) - Constants.Turret.ABS_OFFSET;
        return _positionABS;
    }

    public boolean isTargetInTolerance() {
        return _limelight.isTargetCentered();
    }

    public TurretPose getPose() {
        return new TurretPose(getPositionDegrees());
    }

    public double getManualOffset() {
        return _manualOffset;
    }

    public enum Control {
        Position(0),
        Velocity(1),
        MotionMagic(2),
        PercentOut(3);

        private int _value;

        Control(int value) {_value = value; }

        public int getValue() {
            return _value;
        }
    }

}

