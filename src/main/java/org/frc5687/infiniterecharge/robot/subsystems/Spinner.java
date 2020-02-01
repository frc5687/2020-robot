package org.frc5687.infiniterecharge.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.DriveSpinner;
import org.frc5687.infiniterecharge.robot.subsystems.OutliersSubsystem;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Spinner extends OutliersSubsystem {
    private ColorSensorV3 _colorSensor;
    private CANSparkMax _sparkMax;
    private DoubleSolenoid _solenoid;

    public enum Color {
        red(0),
        green(1),
        blue(2),
        yellow(3),
        unknown(4);

        private int _value;

        Color(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public void raiseArm() {
        _solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void lowerArm(){
        _solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isRaised(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kForward);
    }

    public boolean isLowered(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kReverse);
    }

    public boolean isOff(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kOff);
    }

    public void off(){
        _solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public Spinner(OutliersContainer container) {
        super(container);

        // TODO(mike) do we need to move this to RobotMap? I think we only have one i2c bus?
        I2C.Port port = I2C.Port.kOnboard;
        _colorSensor = new ColorSensorV3(port);
        _sparkMax = new CANSparkMax(RobotMap.CAN.SPARKMAX.SPINNER, CANSparkMaxLowLevel.MotorType.kBrushed);
        setDefaultCommand(new DriveSpinner(this));
    }


    public Color getColor() {
        if (isBlue()) {
            return Color.blue;
        } else if (isRed()) {
            return Color.red;
        } else if (isBlue()) {
            return Color.blue;
        } else if (isGreen()) {
            return Color.green;
        } else if (isYellow()) {
            return Color.yellow;
        }
        return Color.unknown;
    }

    // TODO Fill these in based on measured results.
    public boolean isYellow() {
        return false;
    }

    public boolean isRed() {
        return false;
    }

    public boolean isBlue() {
        return false;
    }

    public boolean isGreen() {
        return false;
    }

    public void spin() {
        _sparkMax.set(Constants.Spinner.SPEED);
    }

    public void stop() {
        _sparkMax.stopMotor();
    }

    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Spinner/Red", _colorSensor.getRed());
        SmartDashboard.putNumber("Spinner/Green", _colorSensor.getGreen());
        SmartDashboard.putNumber("Spinner/Blue", _colorSensor.getBlue());
        SmartDashboard.putNumber("Spinner/IR", _colorSensor.getIR());
        SmartDashboard.putNumber("Spinner/Proximity", _colorSensor.getProximity());
        SmartDashboard.putNumber("Spinner/Color", getColor().getValue());
        SmartDashboard.putNumber("Spinner/SpinnerSpeed", _sparkMax.get());
    }
}
