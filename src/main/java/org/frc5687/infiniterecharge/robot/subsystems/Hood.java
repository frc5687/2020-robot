package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.MedianFilter;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.HallEffect;
import org.frc5687.infiniterecharge.robot.util.Helpers;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Hood extends OutliersSubsystem {

    private TalonSRX _hoodController;
    private Limelight _limelight;
    private HallEffect _hoodHall;
    private OI _oi;

    private double _position;
    private double _setPoint;

    private Limelight.Pipeline _pipeline = Limelight.Pipeline.Wide;

    public Hood(OutliersContainer container, Limelight limelight, OI oi) {
        super(container);
        _limelight = limelight;
        _oi = oi;
        _hoodHall = new HallEffect(RobotMap.DIO.HOOD_HALL);
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
            _hoodController.config_IntegralZone(0, Constants.Hood.I_ZONE, 30);
            _hoodController.selectProfileSlot(0, 0);
        } catch (Exception e) {
            error("Exception allocating hood motor" + e.getMessage());
        }
    }

    public void setSpeed(double speed) {
        _hoodController.set(ControlMode.PercentOutput, speed);
    }

    public void setPosition(double angle) {
        _setPoint = Helpers.limit(angle, Constants.Hood.MIN_DEGREES, Constants.Hood.MAX_DEGREES);
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

    public double getMotorOutput() {
        return _hoodController.getMotorOutputPercent();
    }

    public double getPositionDegrees() {
        return getPositionTicks() * Constants.Hood.TICKS_TO_DEGREES;
    }


    @Override
    public void updateDashboard() {
        metric("Position", getPosition());
        metric("Raw Ticks", getPositionTicks());
        metric("Output Percent", getMotorOutput());
        metric("Limelight Lens Height", getLimelightHeight());
        metric("Limelight Lens Angle", getLimelightAngle());
    }


    public double getPosition() {
        return getPositionDegrees();
    }

    public boolean isAtSetpoint() {
        return _hoodController.isMotionProfileFinished();
    }

    public double getHoodDesiredAngle(double distance) {
        return (13.138*Math.log(distance)) - 12.634;
    }

    public void zeroSensors() {
        if (isHallTriggered()) {
            _position = Constants.Hood.MIN_DEGREES / Constants.Hood.TICKS_TO_DEGREES;
            _setPoint = Constants.Hood.MIN_DEGREES;
        }
        _hoodController.setSelectedSensorPosition((int) _position);
    }

    @Override
    public void periodic() {
        if (isHallTriggered()) {
            if (getMotorOutput() < 0) {
                setSpeed(0);
            }
            _hoodController.setSelectedSensorPosition((int) _position);
            _position = Constants.Hood.MIN_DEGREES / Constants.Hood.TICKS_TO_DEGREES;
        }
    }

    public void setPipeline() {
        if (getPositionDegrees() > 62 && _pipeline != Limelight.Pipeline.TwoTimes) {
            _pipeline = Limelight.Pipeline.TwoTimes;
            _limelight.setPipeline(_pipeline);
        } else if (getPositionDegrees() < 62) {
            _pipeline = Limelight.Pipeline.Wide;
            _limelight.setPipeline(_pipeline);
        }
    }

    public double getLimelightHeight() {
        return Constants.Hood.HEIGHT_TO_DECK + ((-0.0014*Math.pow(getPositionDegrees(), 2)) + (0.284*getPositionDegrees()) + 4.1184); //constants taken from excel formula
    }

    public double getLimelightAngle() {
        return (getPositionDegrees() + Constants.Hood.LIMELIGHT_OFFSET_DEGREES) - 90.5;
    }

    public boolean isHallTriggered() {
        return _hoodHall.get();
    }

}
