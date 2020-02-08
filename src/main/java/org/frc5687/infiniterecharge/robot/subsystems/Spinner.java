package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

import java.util.HashMap;
import java.util.Map;
//
//import static com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate.*;
//import static com.revrobotics.ColorSensorV3.ColorSensorResolution.*;
//import static com.revrobotics.ColorSensorV3.GainFactor.*;

public class Spinner extends OutliersSubsystem {
    private ColorSensorV3 _colorSensor;
    private VictorSPX _motorController;
    private DoubleSolenoid _solenoid;
    private Map<Color, Rgb> _swatches = new HashMap<>();
    private  Color _sensedColor = Color.unknown;
    private int _wedgeCount = 0;

    public void setSpeed(double speed) {
        _motorController.set(ControlMode.PercentOutput, speed);
    }

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
        public String toString() {
            switch (_value) {
                case 0:
                    return "red";
                case 1:
                    return "green";
                case 2:
                    return "blue";
                case 3:
                    return "yellow";
                default:
                    return "unknown";
            }
        }
    }

    public Spinner(OutliersContainer container) {
        super(container);

        try {
            debug("allocating spinner color sensor");
            I2C.Port port = I2C.Port.kOnboard;
            _colorSensor = new ColorSensorV3(port);
            //_colorSensor.configureColorSensor(kColorSensorRes20bit, kColorRate25ms, kGain3x);
        } catch (Exception e) {
            error("error allocating color sensor: " + e.getMessage());
            e.printStackTrace();
        }

        try {
            debug("allocating spinner motor controller");
            _motorController = new VictorSPX(RobotMap.CAN.VICTORSPX.SPINNER);
            _motorController.setNeutralMode(NeutralMode.Brake);
            _solenoid = new DoubleSolenoid(RobotMap.PCM.SPINNER_DEPLOY, RobotMap.PCM.SPINNER_STOW);
            // TODO: Not sure if this is really what we want, just stole from turret...
            // _motorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
        } catch (Exception e) {
            error("error allocating spinner motor controller: " + e.getMessage());
            e.printStackTrace();
        }

        // We sample the color sensor at a higher sample rate than the normal "robot frequency" of 20ms. This
        // might be overkill right now because our color sensor isn't terribly speedy. We could be even more clever
        // and calculate how many wedges went by based on the color we're reading now vs the color we read previously,
        // but I'm already worried that this won't be performant enough as things are.
        new Notifier(() -> {
            edu.wpi.first.wpilibj.util.Color sensorReading = _colorSensor.getColor();
            Color newColor = matchColor(sensorReading);
            if (!newColor.equals(_sensedColor)) {
                _wedgeCount++;
            }
            _sensedColor = newColor;
        }).startPeriodic(Constants.Spinner.SENSOR_SAMPLE_PERIOD_SECONDS);

        // TODO(mike) might want to move to Constants.java ?
        _swatches.put(Color.red, new Rgb(0.60, 0.31, 0.08));
        _swatches.put(Color.yellow, new Rgb(0.40, 0.49, 0.10));
        _swatches.put(Color.green, new Rgb(0.21, 0.56, 0.23));
        _swatches.put(Color.blue, new Rgb(0.21, 0.45, 0.35));
    }

    public Color getColor() {
        return _sensedColor;
    }

    private Color matchColor(edu.wpi.first.wpilibj.util.Color sensorReading) {
        if (closeEnoughTo(sensorReading, _swatches.get(Color.red))) {
            return Color.red;
        } else if (closeEnoughTo(sensorReading, _swatches.get(Color.green))) {
            return Color.green;
        } else if (closeEnoughTo(sensorReading, _swatches.get(Color.blue))) {
            return Color.blue;
        } else if (closeEnoughTo(sensorReading, _swatches.get(Color.yellow))) {
            return Color.yellow;
        }
        return Color.unknown;
    }

    public void deploy() {
        _solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void stow(){
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

    public void spin() {
        _motorController.set(ControlMode.PercentOutput, Constants.Spinner.MOTOR_PERCENT_SPEED);
    }

    public void slow() {
        _motorController.set(ControlMode.PercentOutput, Constants.Spinner.MOTOR_SLOW_PERCENT_SPEED);
    }

    public void stop() {
        _motorController.set(ControlMode.PercentOutput, 0);
    }

    public void resetWedgeCount() {
        _wedgeCount = 0;
    }

    public int getWedgeCount() {
        return _wedgeCount;
    }

    private boolean closeEnoughTo(edu.wpi.first.wpilibj.util.Color sensorReading, Rgb swatch) {
        return Math.abs(sensorReading.red - swatch.red) <= Constants.Spinner.COLOR_TOLERANCE &&
                Math.abs(sensorReading.green - swatch.green) <= Constants.Spinner.COLOR_TOLERANCE &&
                Math.abs(sensorReading.blue - swatch.blue) <= Constants.Spinner.COLOR_TOLERANCE;
    }

    @Override
    public void updateDashboard() {
        metric("Spinner/RawRed", _colorSensor.getRed());
        metric("Spinner/RawGreen", _colorSensor.getGreen());
        metric("Spinner/RawBlue", _colorSensor.getBlue());
        metric("Spinner/NormRed", _colorSensor.getColor().red);
        metric("Spinner/NormGreen", _colorSensor.getColor().green);
        metric("Spinner/NormBlue", _colorSensor.getColor().blue);
        metric("Spinner/Color", getColor().toString());
        metric("Spinner/IR", _colorSensor.getIR());
        metric("Spinner/Proximity", _colorSensor.getProximity());
        metric("Spinner/WedgeCount", _wedgeCount);
//        metric("Spinner/ArmIsRaised", isRaised());
//        metric("Spinner/ArmIsLowered", isLowered());
//        metric("Spinner/ArmIsOff", isArmOff());
//        metric("Spinner/SpinnerSpeed", _motorController.getSelectedSensorVelocity()); // units per 100ms
    }
}
