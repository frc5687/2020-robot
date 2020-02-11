package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.Robot;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

import java.util.HashMap;
import java.util.Map;

public class Spinner extends OutliersSubsystem {
    private ColorSensorV3 _colorSensor;
    private VictorSPX _motorController;
    private DoubleSolenoid _solenoid;
    private Map<Color, Rgb> _swatches = new HashMap<>();
    private Color _previouslySensedColor = Color.unknown;
    private Notifier _sampleTask;
    private int _wedgeCount = 0;

    public Spinner(OutliersContainer container) {
        super(container);

        try {
            debug("allocating spinner color sensor");
            I2C.Port port = I2C.Port.kOnboard;
            _colorSensor = new ColorSensorV3(port);
            _colorSensor.configureColorSensor(
                    ColorSensorV3.ColorSensorResolution.kColorSensorRes16bit,
                    ColorSensorV3.ColorSensorMeasurementRate.kColorRate25ms,
                    ColorSensorV3.GainFactor.kGain3x
             );
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

        // We poll the color sensor to determine if it sufficiently matches one of our swatch colors. If we
        // detect that it has changed since the last sample, we increment the wedge count, which can be used
        // for autospin. Eventually we might want to add some filtering to the callback (especially if we use
        // a sensor with a higher sample rate), or comparison to check whether the read wedge color is the
        // color that we expected to see next. We start this task when we start spinning, and stop it when we
        // stop spinning, so we don't steal too many CPU cycles when thw robot is busy doing other stuff.
        _sampleTask = new Notifier(() -> {
            Color newColor = senseColor();
            if (!newColor.equals(_previouslySensedColor)) {
                _wedgeCount++;
            }
            _previouslySensedColor = newColor;
        });

        // If you're testing color matching you'll want to run this task all the time, instead of only when it's
        // spinning:
        if (Robot.identityMode == Robot.IdentityMode.programming) {
            _sampleTask.startPeriodic(Constants.Spinner.SENSOR_SAMPLE_PERIOD_SECONDS);
        }

        // TODO(mike) might want to move to Constants.java ?
        _swatches.put(Color.red, new Rgb(0.60, 0.31, 0.08));
        _swatches.put(Color.yellow, new Rgb(0.40, 0.49, 0.10));
        _swatches.put(Color.green, new Rgb(0.21, 0.56, 0.23));
        _swatches.put(Color.blue, new Rgb(0.21, 0.45, 0.35));
    }

    /**
     * If a valid color has been cached (e.g., by the periodic notifier task), return that value. Otherwise take a
     * reading from the sensor and attempt to match it.
     * @return Color.red, Color.green, Color.blue, Color.yellow, or Color.unknown
     */
    public Color getColor() {
        if (_previouslySensedColor == null || _previouslySensedColor.equals(Color.unknown)) {
            _previouslySensedColor = senseColor();
        }
        return _previouslySensedColor;
    }

    /**
     * Gets a sensor reading from the color sensor and attempts to match it against the
     * swatches of red, green, yellow, or blue
     * @return Color.red, Color.green, Color.blue, Color.yellow, or Color.unknown
     */
    public Color senseColor() {
        edu.wpi.first.wpilibj.util.Color sensorReading = _colorSensor.getColor();
        return matchColor(sensorReading);
    }

    /**
     * Deploys the spinner motor.
     */
    public void deploy() {
        _solenoid.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Retracts the spinner motor.
     */
    public void stow(){
        _solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isDeployed(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kForward);
    }

    public boolean isStowed(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kReverse);
    }

    public boolean isArmOff(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kOff);
    }

    public void armOff() {
        _solenoid.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * Spins the spinner motor at regular speed.
     */
    public void spin() {
        setSpeed(Constants.Spinner.MOTOR_PERCENT_SPEED);
    }

    /**
     * Spins the spinner motor at low speed.
     */
    public void slow() {
        setSpeed(Constants.Spinner.MOTOR_SLOW_PERCENT_SPEED);
    }

    /**
     * Stops the spinner motor.
     */
    public void stop() {
        _motorController.set(ControlMode.PercentOutput, 0);
    }

    public void setSpeed(double speed) {
        _motorController.set(ControlMode.PercentOutput, speed);
        if (speed != 0) {
            _sampleTask.startPeriodic(Constants.Spinner.SENSOR_SAMPLE_PERIOD_SECONDS);
        } else {
            _sampleTask.stop();
        }
    }

    public void resetWedgeCount() {
        _wedgeCount = 0;
    }

    public int getWedgeCount() {
        return _wedgeCount;
    }

    /**
     * Checks to see if the sensor read color matches any of our swatches.
     * @param sensorReading The raw reading from the color sensor
     * @return Color.red, Color.green, Color.blue, Color.yellow, or Color.unknown
     */
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

    private boolean closeEnoughTo(edu.wpi.first.wpilibj.util.Color sensorReading, Rgb swatch) {
        return Math.abs(sensorReading.red - swatch.red) <= Constants.Spinner.COLOR_TOLERANCE &&
                Math.abs(sensorReading.green - swatch.green) <= Constants.Spinner.COLOR_TOLERANCE &&
                Math.abs(sensorReading.blue - swatch.blue) <= Constants.Spinner.COLOR_TOLERANCE;
    }

    @Override
    public void updateDashboard() {
        if (_colorSensor != null) {
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
        }
        if (_solenoid != null) {
            metric("Spinner/ArmIsRaised", isDeployed());
            metric("Spinner/ArmIsLowered", isStowed());
            metric("Spinner/ArmIsOff", isArmOff());
            metric("Spinner/SpinnerSpeed", _motorController.getSelectedSensorVelocity()); // units per 100ms
        }
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
}
