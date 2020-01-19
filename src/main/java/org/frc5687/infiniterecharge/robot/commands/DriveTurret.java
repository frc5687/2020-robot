package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsytems.Turret;
import org.frc5687.infiniterecharge.robot.util.Limelight;

public class DriveTurret extends OutliersCommand {
    private Turret _turret;
    private Limelight _limelight;
    private OI _oi;

    private double _turretPosition; // in degrees.

    public DriveTurret(Turret turret, Limelight limelight, OI oi) {
        _turret = turret;
        _limelight = limelight;
        _oi = oi;

        addRequirements(_turret);
    }

    @Override
    public void initialize() {
        super.initialize();
        _turret.zeroSensors();
    }

    @Override
    public void execute() {
        double turretSpeed = _oi.getTurretSpeed();
        _turretPosition = _turret.getPositionDegrees();
        if (_turretPosition >= 90 && (turretSpeed > 0)) {
            turretSpeed = 0;
        } else if (_turretPosition <= -180 && (turretSpeed < 0)) {
            turretSpeed = 0;
        }
        metric("AutoPressed", _oi.isAutoTargetPressed());
        if (_oi.isAutoTargetPressed()) {
            _limelight.enableLEDs();
            _turret.setSpeed(getTurnSpeed());
        } else {
            _limelight.disableLEDs();
            _turret.setSpeed(turretSpeed);
        }

    }

    protected double getTurnSpeed() {
        double distance = _limelight.getTargetDistance();
        double angle = _limelight.getHorizontalAngle();
        double targetAngle = angle - _turretPosition;
        metric("targetAngle", targetAngle);
        metric("TurretAngle", _turretPosition);
        metric("limelight angle", angle);
        return angle * 0.07;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
