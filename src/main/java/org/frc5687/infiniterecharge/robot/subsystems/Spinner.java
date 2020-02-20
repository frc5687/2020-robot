package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

import java.util.HashMap;
import java.util.Map;
import java.lang.String;

public class Spinner extends OutliersSubsystem {
    // TODO(mike): Move to Constants somehow?
    private final Color RED = ColorMatch.makeColor(0.50, 0.38, 0.13);
    private final Color GREEN = ColorMatch.makeColor(0.19, 0.55, 0.25);
    private final Color BLUE = ColorMatch.makeColor(0.19, 0.50, 0.40);
    private final Color YELLOW = ColorMatch.makeColor(0.37, 0.51, 0.12);

    private ColorSensorV3 _colorSensor;
    private VictorSPX _motorController;
    private DoubleSolenoid _solenoid;
    private ColorMatch _revColorMatcher = new ColorMatch();
    private Map<MatchedColor, Color> _swatches = new HashMap<>();
    private MatchedColor _previouslyMatchedColor = MatchedColor.unknown;
    private Map<MatchedColor, MatchedColor> _fieldToRobotColorMap = new HashMap<>();
    private Notifier _sampleTask;
    private int _wedgeCount = 0;

    public MatchedColor getTargetColorFromField() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    return MatchedColor.blue;
                case 'G':
                    return  MatchedColor.green;
                case 'R':
                    return MatchedColor.red;
                case 'Y':
                    return MatchedColor.yellow;
                default:
                    error("Corrupt/unknown color returned from field: " + gameData);
            }
        }
        return MatchedColor.unknown;
    }

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
            debug("allocating spinner motor controller and solenoid");
            _motorController = new VictorSPX(RobotMap.CAN.VICTORSPX.SPINNER_SKYWALKER);
            _motorController.setNeutralMode(NeutralMode.Brake);
            _solenoid = new DoubleSolenoid(RobotMap.PCM.SPINNER_DEPLOY, RobotMap.PCM.SPINNER_STOW);
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
            MatchedColor newMatchedColor = senseColor();
            if (!newMatchedColor.equals(_previouslyMatchedColor) && !newMatchedColor.equals(MatchedColor.unknown)) {
                _wedgeCount++;
                _previouslyMatchedColor = newMatchedColor;
            }
        });
        _sampleTask.startPeriodic(Constants.Spinner.SENSOR_SAMPLE_PERIOD_SECONDS);

        _fieldToRobotColorMap.put(MatchedColor.yellow, MatchedColor.green);
        _fieldToRobotColorMap.put(MatchedColor.red, MatchedColor.blue);
        _fieldToRobotColorMap.put(MatchedColor.green, MatchedColor.yellow);
        _fieldToRobotColorMap.put(MatchedColor.blue, MatchedColor.red);

        setupColorMatchingAlgorithms();
    }

    /**
     * If a valid color has been cached (e.g., by the periodic notifier task), return that value. Otherwise take a
     * reading from the sensor and attempt to match it.
     * @return Color.red, Color.green, Color.blue, Color.yellow, or Color.unknown
     */
    public MatchedColor getColor() {
        if (Constants.Spinner.ASYNC_COLOR_SAMPLING) {
            if (_previouslyMatchedColor == null || _previouslyMatchedColor.equals(MatchedColor.unknown)) {
                _previouslyMatchedColor = senseColor();
            }
            return _previouslyMatchedColor;
        }
        return senseColor();
    }

    /**
     * Gets a sensor reading from the color sensor and attempts to match it against the
     * swatches of red, green, yellow, or blue
     * @return Color.red, Color.green, Color.blue, Color.yellow, or Color.unknown
     */
    public MatchedColor senseColor() {
        Color sensorReading = _colorSensor.getColor();
        return matchColor(sensorReading);
    }

    /**
     * The field's sensor is not in the same place as the robot's sensor... it's about 90 degrees away and thus
     * is looking at a completely different color/wedge. This method determines what color the robot will see
     * for a given color that the field sees (incidentally, this function also just happens to work for the
     * inverse case: getting the color the field sees for the provided color the robot sees, because of how
     * the pattern on the control panel works).
     * @param seenByField The color seen by the field's sensor.
     * @return The color the robot should see when the field sees _seenByField_
     */
    public MatchedColor getColorTheRobotSeesForColorTheFieldSees(MatchedColor seenByField) {
        return _fieldToRobotColorMap.getOrDefault(seenByField, MatchedColor.unknown);
    }

    /**
     * Deploys the spinner motor.
     */
    public void deploy() {
        _solenoid.set(DoubleSolenoid.Value.kForward);
        info("Deployed spinner");
        if (Constants.Spinner.ASYNC_COLOR_SAMPLING) {
            _sampleTask.startPeriodic(Constants.Spinner.SENSOR_SAMPLE_PERIOD_SECONDS);
        }
    }

    /**
     * Retracts the spinner motor.
     */
    public void stow() {
        _solenoid.set(DoubleSolenoid.Value.kReverse);
        info("Stowed spinner");
        if (Constants.Spinner.ASYNC_COLOR_SAMPLING) {
            _sampleTask.stop();
        }
    }

    public boolean isDeployed() {
        return _solenoid.get().equals(DoubleSolenoid.Value.kForward);
    }

    public boolean isStowed() {
        return _solenoid.get().equals(DoubleSolenoid.Value.kReverse);
    }

    public boolean isArmOff() {
        return _solenoid.get().equals(DoubleSolenoid.Value.kOff);
    }

    public void armOff() {
        _solenoid.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * Spins the spinner motor at regular speed.
     */
    public void spin() {
        info("Spinning forward at full speed.");
        setSpeed(Constants.Spinner.MOTOR_PERCENT_SPEED);
    }

    /**
     * Spins the spinner motor at low speed.
     */
    public void spinSlowly() {
        info("Spinning forward at low speed.");
        setSpeed(Constants.Spinner.MOTOR_SLOW_PERCENT_SPEED);
    }

    /**
     * Spins the spinner motor backwards at full speed.
     */
    public void spinBackwards() {
        info("Spinning backwards at full speed.");
        _motorController.set(ControlMode.PercentOutput, -1 * Constants.Spinner.MOTOR_PERCENT_SPEED);
    }

    public void spinBackwardsSlowly() {
        info("Spinning backwards at low speed.");
        _motorController.set(ControlMode.PercentOutput, -1 * Constants.Spinner.MOTOR_SLOW_PERCENT_SPEED);
    }

    /**
     * Stops the spinner motor.
     */
    public void stop() {
        info("Stopping the spinner.");
        _motorController.set(ControlMode.PercentOutput, 0);
    }

    public void setSpeed(double speed) {
        _motorController.set(ControlMode.PercentOutput, speed);
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
    private MatchedColor matchColor(edu.wpi.first.wpilibj.util.Color sensorReading) {
        if (Constants.Spinner.USE_HOMEMADE_COLOR_MATCHING_ALGORITHM) {
            // Use crappy algorithm from before we discovered the one in REV's SDK...
            if (closeEnoughTo(sensorReading, _swatches.get(MatchedColor.red))) {
                return MatchedColor.red;
            } else if (closeEnoughTo(sensorReading, _swatches.get(MatchedColor.green))) {
                return MatchedColor.green;
            } else if (closeEnoughTo(sensorReading, _swatches.get(MatchedColor.blue))) {
                return MatchedColor.blue;
            } else if (closeEnoughTo(sensorReading, _swatches.get(MatchedColor.yellow))) {
                return MatchedColor.yellow;
            }
        } else {
            // Use REV Robotics' distance-based algorithm from their SDK...
            _revColorMatcher.setConfidenceThreshold(Constants.Spinner.REV_ALOGORITHM_CONFIDENCE_FACTOR);
            ColorMatchResult match = _revColorMatcher.matchClosestColor(sensorReading);
            if (match.color.equals(RED)) {
                return MatchedColor.red;
            } else if (match.color.equals(GREEN)) {
                return MatchedColor.green;
            } if (match.color.equals(BLUE)) {
                return MatchedColor.blue;
            } if (match.color.equals(YELLOW)) {
                return MatchedColor.yellow;
            }
        }
        return MatchedColor.unknown;
    }

    private boolean closeEnoughTo(edu.wpi.first.wpilibj.util.Color sensorReading, Color swatch) {
        return Math.abs(sensorReading.red - swatch.red) <= Constants.Spinner.COLOR_TOLERANCE &&
                Math.abs(sensorReading.green - swatch.green) <= Constants.Spinner.COLOR_TOLERANCE &&
                Math.abs(sensorReading.blue - swatch.blue) <= Constants.Spinner.COLOR_TOLERANCE;
    }

    private void setupColorMatchingAlgorithms() {
        // Setup the colors to match for our homemade algorithm...
        _swatches.put(MatchedColor.red, RED);
        _swatches.put(MatchedColor.yellow, YELLOW);
        _swatches.put(MatchedColor.green, GREEN);
        _swatches.put(MatchedColor.blue, BLUE);

        // Setup the color to match for REV's distance algorithm...
        _revColorMatcher.addColorMatch(RED);
        _revColorMatcher.addColorMatch(YELLOW);
        _revColorMatcher.addColorMatch(GREEN);
        _revColorMatcher.addColorMatch(BLUE);
    }

    @Override
    public void updateDashboard() {
        if (_colorSensor != null) {
            metric("Spinner/ColorTheFieldSees", getColorTheRobotSeesForColorTheFieldSees(getColor()).toString());
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

    public enum MatchedColor {
        red(0),
        green(1),
        blue(2),
        yellow(3),
        unknown(4);

        private int _value;
        MatchedColor(int value) {
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
