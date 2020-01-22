package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.DriveTurret;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Turret extends OutliersSubsystem {

    private TalonSRX _turretController;
    private Limelight _limelight;
    private DriveTrain _driveTrain;
    private OI _oi;

    private int _positionPIDSlot = 0;
    private int _velocityPIDSlot = 1;
    private int _motionMagicPIDSlot = 2;
    private double _positionABS;
    private double _position;



    public Turret(OutliersContainer container, DriveTrain driveTrain, Limelight limelight, OI oi) {
        super(container);
        _driveTrain = driveTrain;
        _limelight = limelight;
        _oi = oi;

        try {
            debug("allocating turret motor");
            _turretController = new TalonSRX(RobotMap.CAN.TALONSRX.TURRET);
            _turretController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
            _turretController.setSensorPhase(Constants.Turret.SENSOR_PHASE_INVERTED);
            _turretController.configMotionCruiseVelocity(Constants.Turret.CRUISE_VELOCITY);
            _turretController.configMotionAcceleration(Constants.Turret.ACCELERATION);
            _turretController.configVoltageMeasurementFilter(8);
            _turretController.enableVoltageCompensation(true);
            _turretController.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10,20);
            _turretController.configClosedloopRamp(0,50);

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

    public void setMotionMagicSpeed(double output) {
        _turretController.set(ControlMode.MotionMagic, output);
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

    @Override
    public void periodic() {
        setDefaultCommand(new DriveTurret(this,_driveTrain, _limelight, _oi));
    }

    @Override
    public void updateDashboard() {
        metric("Position Ticks", getPositionTicks());
        metric("Position Degrees", getPositionDegrees());
        metric("Absolute Position raw" , getAbsoluteEncoderRawPosition());
        metric("Absolute Pos", getAbsoluteEncoderPosition());
        metric("Velocity", getVelocityTicksPer100ms());
        metric("Speed", _turretController.getMotorOutputVoltage());
    }

    public void zeroSensors() {
        _position = _positionABS;
        _position = _position/Constants.Turret.TICKS_TO_DEGREES;
        _turretController.setSelectedSensorPosition((int) _position);
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

    public enum Control {
        Position(0),
        Velocity(1),
        MotionMagic(2);

        private int _value;

        Control(int value) {_value = value; }

        public int getValue() {
            return _value;
        }
    }

}

