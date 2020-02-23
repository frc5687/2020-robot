package org.frc5687.infiniterecharge.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.commands.IntakeSpin;
import org.frc5687.infiniterecharge.robot.subsystems.OutliersSubsystem;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Intake extends OutliersSubsystem {

    private CANSparkMax _intakeSpark;
    private DoubleSolenoid _intakeSolenoid;
    private OI _oi;

    public Intake(OutliersContainer container, OI oi) {
        super(container);
        _oi = oi;
        _intakeSolenoid = new DoubleSolenoid(RobotMap.PCM.INTAKE_HIGH, RobotMap.PCM.INTAKE_LOW);//check if shifter high and shifter low should be changed

        _intakeSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.INTAKE_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _intakeSpark.setInverted(Constants.Intake.INTAKE_MOTOR_INVERTED);
        _intakeSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public boolean isRunning() {
        return getIntakePower()!=0;
    }

    public enum Position {
        UNKNOWN(DoubleSolenoid.Value.kOff),
        HIGH(DoubleSolenoid.Value.kReverse),
        LOW(DoubleSolenoid.Value.kForward);

        private DoubleSolenoid.Value solenoidValue;

        Position(DoubleSolenoid.Value solenoidValue) {
            this.solenoidValue = solenoidValue;
        }

        public DoubleSolenoid.Value getSolenoidValue() {
            return solenoidValue;
        }

    }


    public Intake.Position getPosition() {
        DoubleSolenoid.Value current = _intakeSolenoid.get();
        if (current== Intake.Position.HIGH.getSolenoidValue()) {
            return Intake.Position.HIGH;
        } else if (current== Intake.Position.LOW.getSolenoidValue()) {
            return Intake.Position.LOW;
        }
        return Intake.Position.UNKNOWN;
    }

    @Override
    public void updateDashboard()
    {
        metric("INTAKE POWER", getIntakePower());
        metric("INTAKE POSITION", getPosition()== Intake.Position.HIGH ? "Intake from Human Player" : (getPosition() == Intake.Position.LOW ? "Intake from Ground" : "Unknown"));
        metric("intakeing", isIntaking());
    }

    public boolean isLowered() {
        return _intakeSolenoid.get() == DoubleSolenoid.Value.kForward;
    }
    public boolean isRaised() {
        return _intakeSolenoid.get() == DoubleSolenoid.Value.kReverse;
    }

    public void raiseIntake() { _intakeSolenoid.set(DoubleSolenoid.Value.kReverse); }

    public void lowerIntake() { _intakeSolenoid.set(DoubleSolenoid.Value.kForward); }

    /*public void updateDashboard() {
        metric("Intake Position", getPosition()== Intake.Position.HIGH ? "Intake from Human Player" : (getPosition() == Intake.Position.LOW ? "Intake from Ground" : "Unknown"));
    }*/


    public void setSpeed(double speed) {
        _intakeSpark.set(speed);
    }

    public double getIntakePower() {return _intakeSpark.get(); }

    public boolean isIntaking() {
        return getIntakePower() > 0;
    }

}