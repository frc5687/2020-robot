package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * Created by Ben Bernard on 2/15/2018.
 */
public class AnglePotentiometer {
    private double _bottomValue;
    private double _minAngle;
    private AnalogPotentiometer _potentiometer;
    private double _scale;
    private double _maxAngle;
    private double _topValue;



    public AnglePotentiometer(int channel, double bottomAngle, double bottomValue, double topAngle, double topValue) {
        _scale = (topAngle - bottomAngle) / (topValue - bottomValue) ;
        _minAngle = bottomAngle;
        _maxAngle = topAngle;
        _bottomValue = bottomValue;


        _potentiometer =  new AnalogPotentiometer(channel);
    }
    public void recalibrate (double bottomValue, double topValue){
        _scale = (_maxAngle - _minAngle) / (topValue - bottomValue) ;
        _bottomValue = bottomValue;
    }

    public void restTop () {
        double topValue = getRaw();
        _scale = (_maxAngle - _minAngle) / (topValue - _bottomValue);
        _topValue = topValue;
    }
    public void restBottem () {
        double bottomValue = getRaw();
        _scale = (_maxAngle - _minAngle) / (_topValue - bottomValue);
        _bottomValue = bottomValue;
    }


    public double get() {
        return _minAngle + (_potentiometer.get() - _bottomValue) * _scale;
    }

    public double getRaw() {
        return _potentiometer.get();
    }

    public double pidGet() {
        return get();
    }
}
