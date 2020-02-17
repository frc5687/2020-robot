package org.frc5687.infiniterecharge.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Lights extends OutliersSubsystem {

    private Spark _shortController;
    private Spark _longController;

    private boolean _controlPanelDetected;
    private boolean _readyToshoot;
    private boolean _targetingButnotReadytoShoot;
    private boolean _intakeDeployedandRunning;
    private boolean _hopperFull;
    private boolean _defult;
    private OI _oi;

    public Lights(OutliersContainer container, OI oi) {
        super(container);
        _shortController = new Spark(RobotMap.PWM.SHORT_LED_STRIP);
        _longController = new Spark(RobotMap.PWM.LONG_LED_STRIP);
        _oi = oi;
    }

    public void setControlPanelDetected(boolean value) {
        _controlPanelDetected = value;
    }

    public void set_readyToshoot (boolean value) {
        _readyToshoot = value;
    }

    public void set_targetingButnotReadytoShoot (boolean value) {
        _targetingButnotReadytoShoot = value;
    }

    public void set_intakeDeployedandRunning (boolean value) {
        _intakeDeployedandRunning = value;
    }

    public void set_hopperFull (boolean value) {
        _hopperFull = value;
    }

    @Override
    public void periodic() {
        Color color = Color.blue;

        if (_controlPanelDetected) {
            color = Color.red;
        } else if (_readyToshoot) {
            color = Color.green;
        } else if (_targetingButnotReadytoShoot) {
            color = Color.yellow;
        } else if (_intakeDeployedandRunning) {
            color = Color.purple;
        } else if (_hopperFull) {
            color = Color.white;
        }

        _shortController.set(color.getBlinkinColor());
        _longController.set(color.getBlinkinColor());
        _oi.setConsoleColors(color.getR(), color.getG(), color.getB());
        metric("Color", color.toString());
    }

    @Override
    public void updateDashboard() {

    }


    private enum Color {
        red(Constants.Lights.SOLID_RED, 1, 0 ,0),
        purple(Constants.Lights.SOLID_PURPLE, 1, 0 ,1 ),
        blue(Constants.Lights.SOLID_BLUE, 0, 0, 1),
        green(Constants.Lights.SOLID_GREEN, 0, 1,0),
        yellow(Constants.Lights.SOLID_YELLOW, 0, 1, 1),
        white(Constants.Lights.SOLID_WHITE, 1 , 1, 1);

        private double _blinkinValue;
        private int _r;
        private int _g;
        private int _b;

        Color(double blinkin, int r, int g, int b) {
            _blinkinValue = blinkin;
            _r = r;
            _g = g;
            _b = b;
        }

        public double getBlinkinColor() { return _blinkinValue; }

        public int getR() { return _r; }
        public int getG() { return _g; }
        public int getB() { return _b; }
    }
}
