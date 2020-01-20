package org.frc5687.infiniterecharge.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    private OI _oi;

    private double _positionABS;
    private double _position;



    public Turret(OutliersContainer container, Limelight limelight, OI oi) {
        super(container);
        _limelight = limelight;
        _oi = oi;

        try {
            debug("allocating turret motor");
            _turretController = new TalonSRX(RobotMap.CAN.TALONSRX.TURRET);
            _turretController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
            _turretController.setSensorPhase(Constants.Turret.SENSOR_PHASE_INVERTED);

        } catch (Exception e) {
            error("error allocating turret motors " + e.getMessage());
        }
        zeroSensors();
    }

    public void setSpeed(double speed) {
        _turretController.set(ControlMode.PercentOutput, speed);
    }

    public void enableBreakMode() {
        _turretController.setNeutralMode(NeutralMode.Brake);
    }

    public void disableBreakMode() {
        _turretController.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void periodic() {
        setDefaultCommand(new DriveTurret(this, _limelight, _oi));
    }

    @Override
    public void updateDashboard() {
        metric("Position Ticks", getPositionTicks());
        metric("Position Degrees", getPositionDegrees());
        metric("Absolute Position raw" , getAbsoluteEncoderRawPosition());
        metric("Absolute Pos", getAbsoluteEncoderPosition());
        metric("Velocity", getVelocityTicksPer100ms());
    }

    public void zeroSensors() {
        _position = _positionABS;
        _turretController.setSelectedSensorPosition((int)_position);
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
        _positionABS = (getAbsoluteEncoderRawPosition() * Constants.Turret.TICKS_TO_DEGREES) - 159;
        return _positionABS;
    }

}

