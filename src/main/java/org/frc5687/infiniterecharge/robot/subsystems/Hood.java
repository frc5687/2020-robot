package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Hood extends OutliersSubsystem {

    private OI _oi;
    private TalonSRX _hoodController;
    private Limelight _limelight;

    private double _positionABS;
    private double _position;
    private double _setPoint;

    private Limelight.Pipeline _pipeline = Limelight.Pipeline.Wide;

    public Hood(OutliersContainer container, Limelight limelight, OI oi) {
        super(container);
        _limelight = limelight;
        _oi = oi;
        _limelight = limelight;
        try {
            debug("Allocating hood motor");
            _hoodController = new TalonSRX(RobotMap.CAN.TALONSRX.HOOD);
            _hoodController.setInverted(Constants.Hood.INVERTED);
            _hoodController.setNeutralMode(NeutralMode.Brake);
            _hoodController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
            _hoodController.setSensorPhase(Constants.Hood.SENSOR_PHASE_INVERTED);
            _hoodController.configForwardSoftLimitThreshold((int)(Constants.Hood.MAX_DEGREES/Constants.Hood.TICKS_TO_DEGREES), 30);
            _hoodController.configForwardSoftLimitEnable(false, 30);
            _hoodController.configReverseSoftLimitThreshold((int) (Constants.Hood.MIN_DEGREES/Constants.Hood.TICKS_TO_DEGREES), 30);
            _hoodController.configReverseSoftLimitEnable(false, 30);
            _hoodController.configMotionCruiseVelocity(Constants.Hood.CRUISE_VELOCITY);
            _hoodController.configMotionAcceleration(Constants.Hood.ACCELERATION);
            _hoodController.configVoltageMeasurementFilter(8);
            _hoodController.enableVoltageCompensation(true);
            _hoodController.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10,20);
            _hoodController.configClosedloopRamp(0,50);
            _hoodController.config_kP(0, Constants.Hood.kP, 30);
            _hoodController.config_kI(0, Constants.Hood.kI, 30);
            _hoodController.config_kD(0, Constants.Hood.kD, 30);
            _hoodController.config_kF(0, Constants.Hood.kF, 30);
            _hoodController.selectProfileSlot(0, 0);
        } catch (Exception e) {
            error("Exception allocating hood motor" + e.getMessage());
        }
        setPosition(getAbsoluteDegrees());
    }

    public void setSpeed(double speed) {
        _hoodController.set(ControlMode.PercentOutput, speed);
    }

    public void setPosition(double angle) {
        _setPoint = angle; // Helpers.limit(angle, Constants.Hood.MIN_DEGREES, Constants.Hood.MAX_DEGREES);
        metric("Setpoint", _setPoint);
        metric("Setpoint Ticks", _setPoint / Constants.Hood.TICKS_TO_DEGREES);
        _hoodController.set(ControlMode.MotionMagic, _setPoint / Constants.Hood.TICKS_TO_DEGREES);
    }

    public int getPositionTicks() {
        return _hoodController.getSelectedSensorPosition(0);
    }

    public double getSetPoint() {
        return _setPoint;
    }

    public double getMotorSpeed() {
        return _hoodController.getMotorOutputPercent();
    }


    public double getPositionDegrees() {
        return getPositionTicks() * Constants.Hood.TICKS_TO_DEGREES;
    }

    public int getPositionAbsoluteRAW() { return
            _hoodController.getSensorCollection().getPulseWidthPosition();}

    public double getAbsoluteDegrees() {
        _positionABS = (getPositionAbsoluteRAW() * Constants.Hood.TICKS_TO_DEGREES) - Constants.Hood.ABS_OFFSET;
        return _positionABS;
    }


    @Override
    public void updateDashboard() {
        metric("Position", getPosition());
        metric("PositionAbs", getAbsoluteDegrees());
        metric("ABS raw", getPositionAbsoluteRAW());
        metric("Raw Ticks", getPositionTicks());
        metric("Output Percent", getMotorSpeed());
    }

    public double getPosition() {
        return getPositionDegrees();
    }

    public boolean isAtSetpoint() {
        return _hoodController.isMotionProfileFinished();
    }

    public double getHoodDesiredAngle(double distance) {
        return distance * Constants.Hood.DISTANCE_ANGLE_CONVERSION;
    }

    public void zeroSensors() {
        _position = _positionABS;
        _position = _position/Constants.Hood.TICKS_TO_DEGREES;
        _hoodController.setSelectedSensorPosition((int) _position);
    }

    public void setPipeline() {

        if (getAbsoluteDegrees() > 60 && _pipeline != Limelight.Pipeline.TwoTimes) {
            _pipeline = Limelight.Pipeline.TwoTimes;
            _limelight.setPipeline(_pipeline);
        } else if (getAbsoluteDegrees() < 60) {
            _pipeline = Limelight.Pipeline.Wide;
            _limelight.setPipeline(_pipeline);
        }
    }
}
