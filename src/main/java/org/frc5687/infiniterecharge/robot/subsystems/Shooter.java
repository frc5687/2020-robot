package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.Helpers;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Shooter extends OutliersSubsystem {

    private DriveTrain _driveTrain;
    private TalonFX _shooterRight;
    private TalonFX _shooterLeft;
    private OI _oi;
    private boolean _shooting = false;
    private double _targetRPM;



    public Shooter(OutliersContainer container, OI oi, DriveTrain driveTrain) {
        super(container);
        _oi = oi;
        _driveTrain = driveTrain;

        _shooterRight = new TalonFX(RobotMap.CAN.TALONFX.RIGHT_SHOOTER);
        _shooterLeft = new TalonFX(RobotMap.CAN.TALONFX.LEFT_SHOOTER);

        _shooterLeft.follow(_shooterRight);
        _shooterRight.config_kP(0,Constants.Shooter.kP, 50);
        _shooterRight.config_kI(0,Constants.Shooter.kI, 50);
        _shooterRight.config_kD(0,Constants.Shooter.kD, 50);
        _shooterRight.config_kF(0,Constants.Shooter.kF, 50);
        _shooterRight.config_IntegralZone(0, Constants.Shooter.I_ZONE, 50);

        _shooterLeft.setInverted(Constants.Shooter.LEFT_INVERTED);
        _shooterRight.setInverted(Constants.Shooter.RIGHT_INVERTED);
        _shooterRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        _shooterRight.getStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        _shooterRight.configClosedloopRamp(1);
        _shooterRight.selectProfileSlot(0,0);
    }

    @Override
    public void updateDashboard() {
        metric("Velocity/Ticks", getVelocity());
        metric("Position", getPosition());
        metric("Velocity/RPM", getRPM());
        metric("Shooting", _shooting);
    }


    public void setShooterSpeed(double speed) {
        metric("Speed", speed);
        _shooterRight.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setVelocitySpeed(double RPM) {
        _targetRPM = RPM;
        _targetRPM = Helpers.limit(_targetRPM, 0, 7200);
        _shooterRight.set(TalonFXControlMode.Velocity, (_targetRPM * Constants.Shooter.TICKS_TO_ROTATIONS / 600 / 1.25));
    }

    public double getPosition() {
        return _shooterRight.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return _shooterRight.getSelectedSensorVelocity();
    }

    public double getRPM() {
        return getVelocity() / Constants.Shooter.TICKS_TO_ROTATIONS * 600 * Constants.Shooter.GEAR_RATIO;
    }

    public boolean isAtVelocity(double RPM) {
        return Math.abs(getRPM() - RPM) < Constants.Shooter.RPM_TOLERANCE;
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(getRPM() - _targetRPM) < Constants.Shooter.RPM_TOLERANCE;
    }

    public double getDistanceSetpoint(double distance) {
        return (-0.0029*(distance*distance)) + (7.5713*distance) + 2209.6;
    }

    public boolean isShooting() {
        return _shooting;
    }

    public void setShooting(boolean shooting) {
        _shooting = shooting;
    }


}
