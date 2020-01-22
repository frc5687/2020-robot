package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;
import org.frc5687.infiniterecharge.robot.util.Limelight;

public class DriveTurret extends OutliersCommand {
    private Turret _turret;
    private DriveTrain _driveTrain;
    private Limelight _limelight;
    private OI _oi;

    private double _turretPosition; // in degrees.


    public DriveTurret(Turret turret,DriveTrain driveTrain, Limelight limelight, OI oi) {
        _turret = turret;
        _driveTrain = driveTrain;
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
        metric("Turret Speed", turretSpeed);
        _turretPosition = _turret.getPositionDegrees();
        if (_turretPosition >= Constants.Turret.MAX_DEGREES && (turretSpeed > 0)) {
            turretSpeed = 0;
        } else if (_turretPosition <= Constants.Turret.MIN_DEGREES && (turretSpeed < 0)) {
            turretSpeed = 0;
        }
        metric("AutoPressed", _oi.isAutoTargetPressed());
        if (_oi.isAutoTargetPressed()) {
//            _limelight.enableLEDs();
            _turret.setSpeed(getTurnSpeedTest());
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

    protected double getTurnSpeedTest() {
        return (_driveTrain.getAngleToTarget() - _turretPosition) * 0.068;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
