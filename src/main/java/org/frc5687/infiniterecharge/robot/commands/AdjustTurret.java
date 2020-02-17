package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.subsystems.Turret;

public class AdjustTurret extends OutliersCommand {
    private Turret _turret;
    private double _amount;

    public AdjustTurret(Turret turret, double amount) {
        _turret = turret;
        _amount = amount;
    }

    @Override
    public void initialize() {
        super.initialize();
        _turret.adjustOffset(_amount);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
