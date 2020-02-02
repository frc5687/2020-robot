package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.DriveSpinner;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

import java.util.HashMap;
import java.util.Map;

public class Spinner extends OutliersSubsystem {
    private ColorSensorV3 _colorSensor;
    private TalonSRX _motorController;
    private DoubleSolenoid _solenoid;
    private Map<Color, Rgb> _swatches = new HashMap<>();

    public static class Rgb {
        private double red;
        private double green;
        private double blue;
        public Rgb(double red, double green, double blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
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

    public boolean isArmOff(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kOff);
    }

    public void armOff() {
        _solenoid.set(DoubleSolenoid.Value.kOff);
    }

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

    public Spinner(OutliersContainer container) {
        super(container);

        try {
            debug("allocating spinner color sensor");
            I2C.Port port = I2C.Port.kOnboard;
            _colorSensor = new ColorSensorV3(port);
        } catch (Exception e) {
            error("error allocating color sensor: " + e.getMessage());
            e.printStackTrace();
        }

        try {
            debug("allocating spinner motor controller");
            _motorController = new TalonSRX(RobotMap.CAN.TALONSRX.SPINNER);
            _motorController.setNeutralMode(NeutralMode.Brake);
            // TODO: Not sure if this is really what we want, just stole from turret...
            _motorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
        } catch (Exception e) {
            error("error allocating spinner motor controller: " + e.getMessage());
            e.printStackTrace();
        }

        setDefaultCommand(new DriveSpinner(this));

        // TODO(mike) might want to move to Constants.java ?
        _swatches.put(Color.red, new Rgb(0.58, 0.31, 0.10));
        _swatches.put(Color.yellow, new Rgb(0.36, 0.53, 0.10));
        _swatches.put(Color.green, new Rgb(0.21, 0.56, 0.23));
        _swatches.put(Color.blue, new Rgb(0.15, 0.43, 0.45));
    }

    public Color getColor() {
        if (closeEnoughTo(_swatches.get(Color.red))) {
            return Color.red;
        } else if (closeEnoughTo(_swatches.get(Color.green))) {
            return Color.green;
        } else if (closeEnoughTo(_swatches.get(Color.blue))) {
            return Color.blue;
        } else if (closeEnoughTo(_swatches.get(Color.yellow))) {
            return Color.yellow;
        }
        return Color.unknown;
    }

    public void spin() {
        _motorController.set(ControlMode.PercentOutput, Constants.Spinner.MOTOR_PERCENT_SPEED);
    }

    public void stop() {
        _motorController.set(ControlMode.PercentOutput, 0);
    }

    private boolean closeEnoughTo(Rgb swatch) {
        edu.wpi.first.wpilibj.util.Color readColor = _colorSensor.getColor();
        return Math.abs(readColor.red - swatch.red) <= Constants.Spinner.COLOR_TOLERANCE &&
                Math.abs(readColor.green - swatch.green) <= Constants.Spinner.COLOR_TOLERANCE &&
                Math.abs(readColor.blue - swatch.blue) <= Constants.Spinner.COLOR_TOLERANCE;
    }

    @Override
    public void updateDashboard() {
        metric("Spinner/RawRed", _colorSensor.getRed());
        metric("Spinner/RawGreen", _colorSensor.getGreen());
        metric("Spinner/RawBlue", _colorSensor.getBlue());
        metric("Spinner/NormRed", _colorSensor.getColor().red);
        metric("Spinner/NormGreen", _colorSensor.getColor().green);
        metric("Spinner/NormBlue", _colorSensor.getColor().blue);
        metric("Spinner/Color", getColor().getValue());
        metric("Spinner/IR", _colorSensor.getIR());
        metric("Spinner/Proximity", _colorSensor.getProximity());
        metric("Spinner/ArmIsRaised", isRaised());
        metric("Spinner/ArmIsLowered", isLowered());
        metric("Spinner/ArmIsOff", isArmOff());
        metric("Spinner/SpinnerSpeed", _motorController.getSelectedSensorVelocity()); // units per 100ms
    }
}
