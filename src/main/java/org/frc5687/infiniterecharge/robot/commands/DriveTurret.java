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
    }

    @Override
    public void execute() {
        double turretSpeed = _oi.getTurretSpeed();
        _turretPosition = _turret.getPosition();
//        if (_turretPosition == 0 && (turretSpeed > 0)) {
//            turretSpeed = 0;
//        } else if (_turretPosition == 270 && (turretSpeed < 0)) {
//            turretSpeed = 0;
//        }

        _turret.setSpeed(turretSpeed);

    }

    protected double getTurnSpeed() {
        double distance = _limelight.getTargetDistance();
        double angle = _limelight.getHorizontalAngle();
        double targetAngle = angle - _turretPosition;
        return targetAngle * 0.0012;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
