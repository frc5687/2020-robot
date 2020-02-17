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

    private boolean _indexerTop;
    private boolean _indexerMiddle;
    private boolean _indexerBottom;


    public Lights(OutliersContainer container, OI oi) {
        super(container);
        _shortController = new Spark(RobotMap.PWM.SHORT_LED_STRIP);
        _longController = new Spark(RobotMap.PWM.LONG_LED_STRIP);
        _oi = oi;
    }

    public void setControlPanelDetected(boolean value) {
        _controlPanelDetected = value;
    }

    public void setReadyToshoot(boolean value) {
        _readyToshoot = value;
    }

    public void setTargeting(boolean value) {
        _targetingButnotReadytoShoot = value;
    }

    public void setAutoIntaking(boolean value) {
        _intakeDeployedandRunning = value;
    }

    public void setHopperFull(boolean value) {
        _hopperFull = value;
    }

    @Override
    public void periodic() {
        Color consoleColor = Color.blue;
        Color shortColor = Color.blue;
        Color longColor = Color.blue;

        if (_controlPanelDetected) {
            shortColor = longColor = consoleColor = Color.red;
        } else if (_readyToshoot) {
            shortColor = longColor = consoleColor = Color.green;
        } else if (_targetingButnotReadytoShoot) {
            shortColor = longColor = consoleColor = Color.yellow;
        } else if (_intakeDeployedandRunning) {
            shortColor = longColor = consoleColor = Color.purple;
        } else if (_indexerBottom && _indexerMiddle && _indexerTop) {
            consoleColor = Color.white;
            shortColor = Color.white;
            longColor = Color.white;
        } else if (_indexerMiddle && _indexerTop || _indexerMiddle && _indexerBottom || _indexerBottom && _indexerTop) {
            consoleColor = Color.white;
            shortColor = Color.black;
            longColor = Color.white;
        } else if (_indexerTop || _indexerMiddle || _indexerBottom) {
            consoleColor = Color.white;
            shortColor = Color.white;
            longColor = Color.black;
        }

        _shortController.set(shortColor.getBlinkinColor());
        _longController.set(longColor.getBlinkinColor());
        _oi.setConsoleColor(consoleColor.getR(), consoleColor.getG(), consoleColor.getB());
        metric("Color", consoleColor.toString());
    }

    @Override
    public void updateDashboard() {

    }

    public void setIndexerBottom(boolean triggered) {
        _indexerBottom = triggered;
    }

    public void setIndexerMiddle(boolean triggered) {
        _indexerMiddle = triggered;
    }

    public void setIndexerTop(boolean triggered) {
        _indexerTop = triggered;
    }


    private enum Color {
        red(Constants.Lights.SOLID_RED, 1, 0 ,0),
        purple(Constants.Lights.SOLID_PURPLE, 1, 0 ,1 ),
        blue(Constants.Lights.SOLID_BLUE, 0, 0, 1),
        green(Constants.Lights.SOLID_GREEN, 0, 1,0),
        yellow(Constants.Lights.SOLID_YELLOW, 0, 1, 1),
        white(Constants.Lights.SOLID_WHITE, 1 , 1, 1),
        black(Constants.Lights.SOLID_BLACK, 0, 0 ,0 );

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
