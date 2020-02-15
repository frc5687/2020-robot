package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Shooter extends OutliersSubsystem {

    private DriveTrain _driveTrain;
    private TalonFX _shooterRight;
    private TalonFX _shooterLeft;
    private OI _oi;


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

        _shooterLeft.setInverted(Constants.Shooter.LEFT_INVERTED);
        _shooterRight.setInverted(Constants.Shooter.RIGHT_INVERTED);
        _shooterRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    @Override
    public void updateDashboard() {
        metric("Velocity/Ticks", getVelocity());
        metric("Position", getPosition());
        metric("Velocity/RPM", getRPM());
    }


    public void setShooterSpeed(double speed) {
        _shooterRight.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setVelocitySpeed(double RPM) {_shooterRight.set(TalonFXControlMode.Velocity, RPM);}

    public double getPosition() {
        return _shooterRight.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return _shooterRight.getSelectedSensorVelocity();
    }

    public double getRPM() {
        return getVelocity() * Constants.Shooter.TICKS_TO_ROTATIONS * 600;
    }


    public boolean isAtVelocity(double RPM) {
        return Math.abs(getVelocity() - RPM) < Constants.Shooter.RPM_TOLERANCE;
    }

    public double getDistanceSetpoint() {
        return _driveTrain.distanceToTarget() * 50;
    }


}
