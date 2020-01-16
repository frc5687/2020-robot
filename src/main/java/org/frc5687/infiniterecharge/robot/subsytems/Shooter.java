package org.frc5687.infiniterecharge.robot.subsytems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.Shoot;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Shooter extends OutliersSubsystem {

    TalonFX _rightShooter;
    TalonFX _leftShooter;
    OI _oi;

    public Shooter(OutliersContainer container, OI oi) {
        super(container);
        _oi = oi;

        _rightShooter = new TalonFX(RobotMap.CAN.TALONFX.RIGHT_SHOOTER);
        _leftShooter = new TalonFX(RobotMap.CAN.TALONFX.LEFT_SHOOTER);

        _leftShooter.follow(_rightShooter);

        _leftShooter.setInverted(Constants.Shooter.LEFT_INVERTED);
        _rightShooter.setInverted(Constants.Shooter.RIGHT_INVERTED);
    }

    @Override
    public void updateDashboard() {
    }

    @Override
    public void periodic() {
        setDefaultCommand(new Shoot(this, _oi));
    }

    public void setSpeed(double speed) {
        _rightShooter.set(TalonFXControlMode.PercentOutput, speed);
    }


    public double getPosition() {
        return _leftShooter.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return _leftShooter.getSelectedSensorVelocity();
    }
}
