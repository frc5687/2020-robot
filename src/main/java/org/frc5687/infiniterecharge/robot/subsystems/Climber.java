package org.frc5687.infiniterecharge.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.Climb;
import org.frc5687.infiniterecharge.robot.commands.IntakeSpin;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Climber extends OutliersSubsystem {
    private CANSparkMax _climberSpark;
    private OI _oi;

    public Climber(OutliersContainer container, OI oi) {
        super(container);
        _oi = oi;

        _climberSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.CLIMBER_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _climberSpark.setInverted(Constants.Climber.CLIMBER_MOTOR_INVERTED);
    }



    @Override
    public void updateDashboard()
    {
        metric("CLIMBER POWER", getClimberPower());
    }

    @Override
    public void periodic() {
        setDefaultCommand(new Climb(this, _oi));
    }



    public void setSpeed(double speed) {
        _climberSpark.set(speed);
    }

    public double getClimberPower() {return _climberSpark.get(); }
}
