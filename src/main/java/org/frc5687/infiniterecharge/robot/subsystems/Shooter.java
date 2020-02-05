package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.Shoot;
import org.frc5687.infiniterecharge.robot.subsystems.OutliersSubsystem;
import org.frc5687.infiniterecharge.robot.util.DigitalIR;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Shooter extends OutliersSubsystem {

    private TalonFX _shooterRight;
    private TalonFX _shooterLeft;
    private OI _oi;

    public Shooter(OutliersContainer container, OI oi) {
        super(container);
        _oi = oi;

        _shooterRight = new TalonFX(RobotMap.CAN.TALONFX.RIGHT_SHOOTER);
        _shooterLeft = new TalonFX(RobotMap.CAN.TALONFX.LEFT_SHOOTER);

        _shooterLeft.follow(_shooterRight);

        _shooterLeft.setInverted(Constants.Shooter.LEFT_INVERTED);
        _shooterRight.setInverted(Constants.Shooter.RIGHT_INVERTED);
    }

    @Override
    public void updateDashboard() {
        metric("Velocity", getVelocity());
    }


    public void setShooterSpeed(double speed) {
        _shooterRight.set(TalonFXControlMode.PercentOutput, speed);
    }


    public double getPosition() {
        return _shooterLeft.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return _shooterLeft.getSelectedSensorVelocity();
    }


}
