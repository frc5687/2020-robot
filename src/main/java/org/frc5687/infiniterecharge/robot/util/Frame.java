package org.frc5687.infiniterecharge.robot.util;

public class Frame {
    private double _velocity;
    private double _omega;
    private float _yaw;

    public Frame(){
        _velocity = 0;
        _omega = 0;
        _yaw = 0;
    }
    public Frame(double velocity, double omega, float yaw)  {
        _velocity = velocity;
        _omega = omega;
        _yaw = yaw;
    }

    public void setVelocity(double velocity){
        _velocity = velocity;
    }
    //angular velocity
    public void setOmega(double omega){
        _omega = omega;
    }
    public void setYaw(float yaw) {
        _yaw = yaw;
    }

    public double getVelocity(){
        return _velocity;
    }

    public double getOmega() {
        return  _omega;
    }
    public float getYaw(){
        return _yaw;
    }
}
