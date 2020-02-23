package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;
import org.frc5687.infiniterecharge.robot.util.Limelight;

public class DriveTurret extends OutliersCommand {
    private Turret _turret;
    private OI _oi;

    public DriveTurret(Turret turret, OI oi) {
        _turret = turret;
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
        _turret.setSpeed(turretSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
