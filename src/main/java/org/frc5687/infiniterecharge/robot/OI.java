package org.frc5687.infiniterecharge.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.AxisButton;
import org.frc5687.infiniterecharge.robot.util.Gamepad;
import org.frc5687.infiniterecharge.robot.util.OutliersProxy;
import org.frc5687.infiniterecharge.robot.util.POV;

import static org.frc5687.infiniterecharge.robot.util.Helpers.applyDeadband;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    protected Button _driverRightStickButton;

    private Button _operatorLeftTrigger;

    private Button _driverRightBumper;
    private Button _driverLeftBumper;

    private Button _operatorAButton;

    private AxisButton _driverRightYAxisUpButton;

    private AxisButton _operatorRightXAxisUpButton;
    private AxisButton _operatorRightXAxisDownButton;


    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _driverRightStickButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _operatorLeftTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);

        _driverRightBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _driverLeftBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());


        _driverRightYAxisUpButton = new AxisButton(_driverGamepad,Gamepad.Axes.RIGHT_Y.getNumber(), -.75);
        _operatorRightXAxisDownButton = new AxisButton(_operatorGamepad,Gamepad.Axes.RIGHT_Y.getNumber(), -.5);
        _operatorRightXAxisUpButton = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber(), .5);

        _operatorAButton = new JoystickButton(_driverGamepad,Gamepad.Buttons.A.getNumber());
    }


    public void initializeButtons(Shifter shifter, DriveTrain driveTrain, Intake intake, Climber climber){
    }

    public boolean isAutoTargetPressed() {
        return _driverLeftBumper.get();
    }

    public double getDriveSpeed() {
        double speed = -getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public double getDriveRotation() {
        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public double getTurretSpeed() {
        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, Constants.Turret.DEADBAND);
        return speed;
    }

    public double getIntakeSpeed() {
        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public double getClimberSpeed() {
        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public double getHoodSpeed() {
        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.Hood.DEADBAND);
        return speed;
    }

    public int getOperatorPOV() {
        return POV.fromWPILIbAngle(0, _operatorGamepad.getPOV()).getDirectionValue();
    }
    public int getDriverPOV() {
        return POV.fromWPILIbAngle(0, _driverGamepad.getPOV()).getDirectionValue();
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {

    }

    private int _driverRumbleCount = 0;
    private int _operatorRumbleCount = 0;
    private long _driverRumbleTime = System.currentTimeMillis();
    private long _operatorRumbleTime = System.currentTimeMillis();

    public void pulseDriver(int count) {
        // Check to see if we are already rumbling!
        if (_driverRumbleCount > 0) { return; }
        _driverRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
        _driverRumbleCount = count * 2;
    }
    public void pulseOperator(int count) {
        // Check to see if we are already rumbling!
        if (_operatorRumbleCount > 0) { return; }
        _operatorRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
        _operatorRumbleCount = count * 2;
    }


    public void poll() {
        if (_driverRumbleCount > 0) {
            _driverGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, _driverRumbleCount % 2 == 0 ? 0 : 1);
            _driverGamepad.setRumble(GenericHID.RumbleType.kRightRumble, _driverRumbleCount % 2 == 0 ? 0 : 1);
            if (System.currentTimeMillis() > _driverRumbleTime) {
                _driverRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
                _driverRumbleCount--;
            }
        } else {
            _driverGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            _driverGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }

        if (_operatorRumbleCount > 0) {
            _operatorGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, _operatorRumbleCount % 2 == 0 ? 0 : 1);
            _operatorGamepad.setRumble(GenericHID.RumbleType.kRightRumble, _operatorRumbleCount % 2 == 0 ? 0 : 1);
            if (System.currentTimeMillis() > _operatorRumbleTime) {
                _operatorRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
                _operatorRumbleCount--;
            }
        } else {
            _operatorGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            _operatorGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }
    }

    public boolean isKillAllPressed() {
        int operatorPOV = getOperatorPOV();
        int driverPOV = getDriverPOV();

        return driverPOV == Constants.OI.KILL_ALL || operatorPOV == Constants.OI.KILL_ALL;
    }
    public boolean isOverridePressed() {
        int operatorPOV = getOperatorPOV();
        int driverPOV = getDriverPOV();

        return driverPOV == Constants.OI.OVERRIDE || operatorPOV == Constants.OI.OVERRIDE;
    }


    public boolean isCreepPressed() {
        return  _driverRightStickButton.get();
    }
}

