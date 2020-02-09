package org.frc5687.infiniterecharge.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.ExtendElevator;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Climber extends OutliersSubsystem {
    private CANSparkMax _elevatorSpark;
    private CANEncoder _elevatorEncoder;

    private CANSparkMax _winchSpark;
    private CANEncoder _winchEncoder;

    private OI _oi;

    public Climber(OutliersContainer container, OI oi) {
        super(container);
        _oi = oi;

        _elevatorSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.ELEVATOR_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _elevatorSpark.setInverted(Constants.Climber.ELEVATOR_MOTOR_INVERTED);
        _elevatorEncoder = _elevatorSpark.getEncoder();

        _winchSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.WINCH_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _winchSpark.setInverted(Constants.Climber.WINCH_MOTOR_INVERTED);
        _winchEncoder = _winchSpark.getEncoder();

    }



    @Override
    public void updateDashboard()
    {
        metric("CLIMBER POWER", getClimberPower());
        metric("CLIMBER POSITION", getPosition());
    }



    public void setElevatorSpeed(double speed) {
        _elevatorSpark.set(speed);
    }

    public void setWinchSpeed(double speed) {
        _winchSpark.set(speed);
    }

    public double getClimberPower() {return _elevatorSpark.get(); }
    public double getPosition() {return _elevatorEncoder.getPosition();}

}
