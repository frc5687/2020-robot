package org.frc5687.infiniterecharge.robot;

import org.frc5687.infiniterecharge.robot.util.BasicPose;
import org.frc5687.infiniterecharge.robot.util.Pose;

public class RobotPose extends BasicPose {
    private double _turretAngle;
    private double _hoodAngle;

    public RobotPose(double angle, double leftEncoder, double rightEncoder, double distance, double turretAngle, double hoodAngle) {
        super(angle, leftEncoder, rightEncoder, distance);
        _turretAngle = turretAngle;
        _hoodAngle = hoodAngle;
    }

    public double getTurretAngle() { return _turretAngle; }

    public double getHoodAngle() { return _hoodAngle; }
}
