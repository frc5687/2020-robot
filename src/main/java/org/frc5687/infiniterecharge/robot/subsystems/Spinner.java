package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

import java.util.HashMap;
import java.util.Map;
import java.lang.String;

public class Spinner extends OutliersSubsystem {
    private ColorSensorV3 _colorSensor;
    private VictorSPX _motorController;
    private DoubleSolenoid _solenoid;
    private Map<Color, Rgb> _swatches = new HashMap<>();
    private Map<Color, Color> _fieldToRobotColorMap = new HashMap<>(); // see getColorRobotSeesForColorFieldSees()
    private boolean FMSDataCorrupt = false;

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
        unknown(4),
        badData(5);

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

    public Color getSoughtColor() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0){
            switch (gameData.charAt(0)){
                case 'B':
                    return Color.blue;
                case 'G':
                    return  Color.green;
                case 'R':
                    return Color.red;
                case 'Y':
                    return Color.yellow;
                default:
                    error("Corrupt/unknown color returned from field: " + gameData);
                    FMSDataCorrupt = true;
                    return Color.unknown;
            }
        }
        return Color.unknown;
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
            _motorController = new VictorSPX(RobotMap.CAN.VICTORSPX.SPINNER);
            _motorController.setNeutralMode(NeutralMode.Brake);
            _solenoid = new DoubleSolenoid(RobotMap.PCM.SPINNER_DEPLOY, RobotMap.PCM.SPINNER_STOW);
            // TODO: Not sure if this is really what we want, just stole from turret...
            // _motorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
        } catch (Exception e) {
            error("error allocating spinner motor controller: " + e.getMessage());
            e.printStackTrace();
        }

        // TODO(mike) might want to move to Constants.java ?
        _swatches.put(Color.red, new Rgb(0.60, 0.31, 0.08));
        _swatches.put(Color.yellow, new Rgb(0.40, 0.49, 0.10));
        _swatches.put(Color.green, new Rgb(0.21, 0.56, 0.23));
        _swatches.put(Color.blue, new Rgb(0.21, 0.45, 0.35));

        _fieldToRobotColorMap.put(Color.yellow, Color.green);
        _fieldToRobotColorMap.put(Color.red, Color.blue);
        _fieldToRobotColorMap.put(Color.green, Color.yellow);
        _fieldToRobotColorMap.put(Color.blue, Color.red);
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

    /**
     * The field's sensor is not in the same place as the robot's sensor... it's about 90 degrees away and thus
     * is looking at a completely different color/wedge. This method determines what color the robot will see
     * for a given color that the field sees (incidentally, this function also just happens to work for the
     * inverse case: getting the color the field sees for the provided color the robot sees, because of how
     * the pattern on the control panel works).
     * @param seenByField The color seen by the field's sensor.
     * @return The color the robot should see when the field sees _seenByField_
     */
    public Color getColorTheRobotSeesForColorTheFieldSees(Color seenByField) {
        return _fieldToRobotColorMap.get(seenByField);
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

    public void spinBackwards() {
        _motorController.set(ControlMode.PercentOutput, -1 * Constants.Spinner.MOTOR_PERCENT_SPEED);
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
        metric("Spinner/ArmIsRaised", isRaised());
        metric("Spinner/ArmIsLowered", isLowered());
        metric("Spinner/ArmIsOff", isArmOff());
        metric("Spinner/SpinnerSpeed", _motorController.getSelectedSensorVelocity()); // units per 100ms
    }
}
