package org.frc5687.infiniterecharge.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.*;

import static org.frc5687.infiniterecharge.robot.util.Helpers.applyDeadband;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Gamepad _operatorGamepad;
    protected Launchpad _launchpad;
    protected Button _driverRightStickButton;

    private Button _operatorLeftTrigger;
    private Button _operatorRightTrigger;
    private Button _driverLeftTrigger;
    private Button _driverRightTrigger;


    private Button _driverRightBumper;
    private Button _driverLeftBumper;

    private Button _operatorRightBumper;
    private Button _operatorLeftBumper;

    private Button _operatorStartButton;
    private Button _operatorEndButton;


    private Button _driverAButton;
    private Button _driverBButton;
    private Button _driverXButton;
    private Button _driverYButton;

    private Button _operatorAButton;
    private Button _operatorBButton;
    private Button _operatorXButton;
    private Button _operatorYButton;

    private AxisButton _driverRightYAxisUpButton;

    private AxisButton _operatorRightXAxisUpButton;
    private AxisButton _operatorRightXAxisDownButton;

    private AxisButton _operatorLeftYAxisUpButton;
    private AxisButton _operatorLeftYAxisDownButton;


    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _launchpad = new Launchpad(2);

        _driverRightStickButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _driverLeftTrigger = new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _driverRightTrigger = new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorRightTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorLeftTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorLeftBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());
        _operatorRightBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());

        _driverRightBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _driverLeftBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());


        _driverRightYAxisUpButton = new AxisButton(_driverGamepad,Gamepad.Axes.RIGHT_Y.getNumber(), -.75);
        _operatorRightXAxisDownButton = new AxisButton(_operatorGamepad,Gamepad.Axes.RIGHT_Y.getNumber(), -.5);
        _operatorRightXAxisUpButton = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber(), .5);

        _driverAButton = new JoystickButton(_driverGamepad,Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad,Gamepad.Buttons.B.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad,Gamepad.Buttons.Y.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad,Gamepad.Buttons.X.getNumber());

        _operatorAButton = new JoystickButton(_operatorGamepad,Gamepad.Buttons.A.getNumber());
        _operatorBButton = new JoystickButton(_operatorGamepad,Gamepad.Buttons.B.getNumber());
        _operatorXButton = new JoystickButton(_operatorGamepad,Gamepad.Buttons.X.getNumber());
        _operatorYButton = new JoystickButton(_operatorGamepad,Gamepad.Buttons.Y.getNumber());

        _operatorStartButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.START.getNumber());
        _operatorEndButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.BACK.getNumber());

        _operatorLeftYAxisDownButton = new AxisButton(_operatorGamepad,Gamepad.Axes.LEFT_Y.getNumber(), -.5);
        _operatorLeftYAxisUpButton = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber(), .5);

    }

    public void initializeButtons(Shifter shifter, DriveTrain driveTrain, Turret turret, Limelight limelight, PoseTracker poseTracker, Intake intake, Shooter shooter, Indexer indexer, Spinner spinner, Climber climber, Hood hood, Skywalker skywalker, Lights lights){
        _operatorRightBumper.whileHeld(new AutoTarget(turret, shooter, hood, limelight, driveTrain, poseTracker, lights,this,5700, 67));
        _operatorLeftBumper.whileHeld(new AutoTarget(turret, shooter, hood, limelight, driveTrain, poseTracker, lights,this,3150, 51));

        _operatorRightTrigger.whenHeld(new Shoot(shooter, indexer, turret, this, 5700));

        _operatorStartButton.whileHeld(new ExtendElevator(climber));
        // _operatorXButton.whileHeld(new RetractElevator(climber));
        _operatorEndButton.whileHeld(new RetractWinch(climber));

//        _operatorXButton.whenPressed(new AdjustTurret(turret, -1));
        _operatorBButton.whenPressed(new ShootSpeedSetpoint(shooter, this, 2200));
        _operatorAButton.whenPressed(new ShootSpeedSetpoint(shooter, this, 2100));

        _operatorXButton.whileHeld(new AutoBalance(skywalker, spinner, this));
        _driverLeftBumper.whenPressed(new Shift(driveTrain, shifter, Shifter.Gear.HIGH, false));
        _driverRightBumper.whenPressed(new Shift(driveTrain, shifter, Shifter.Gear.LOW, false));

        _driverAButton.whenPressed(new AutoTurretSetpoint(turret, driveTrain,limelight,this, 0));
        _driverBButton.whenPressed(new AutoTurretSetpoint(turret, driveTrain,limelight,this, -90));
        _driverYButton.whenPressed(new AutoTurretSetpoint(turret, driveTrain,limelight,this, -180));
        _driverXButton.whenPressed(new AutoTurretSetpoint(turret, driveTrain,limelight,this, 90));
        _operatorLeftTrigger.whileHeld(new AutoIntake(intake, lights));


        _operatorYButton.whileHeld(new DriveSpinner(spinner, this));
        //_operatorYButton.whenPressed(new AutoSpinRotations(spinner, this, skywalker));  // <~ When we're ready, this works!
    }

    public boolean isAutoTargetPressed() {
        return false;
    }
    public boolean isAutoTargetDrivePressed() {return _driverRightYAxisUpButton.get();}

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

    public double getShooterSpeed() {

        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber());
        speed = applyDeadband(speed, Constants.Shooter.DEADBAND);
        return speed;
    }

    public double getIndexerSpeed() {
        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, Constants.Shooter.DEADBAND);
        return speed;
    }

    public double getSkywalkerSpeed() {
        //if (getSubSystem()!=SubSystem.Skywalker) { return 0; }
        double speed = getSpeedFromAxis(_driverGamepad, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, Constants.Skywalker.DEADBAND);
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


    public double getHoodSpeed() {
        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber());
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

    public double getSpinnerSpeed() {
        return getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_X.getNumber());
    }

    private enum SubSystem {
        None,
        Intake,
        Shooter,
        Spinner,
        Climber
    }

    public void setConsoleColor(int red, int green, int blue) {
        setConsoleColor(red > 0, green > 0, blue > 0);
    }

    public void setConsoleColor(boolean red, boolean green, boolean blue) {
        if (_launchpad==null) { return; }
        try {
            _launchpad.setOutput(Constants.OI.RED_CHANNEL, red);
            _launchpad.setOutput(Constants.OI.GREEN_CHANNEL, green);
            _launchpad.setOutput(Constants.OI.BLUE_CHANNEL, blue);
        } catch (Exception e) {
        }
    }

}

