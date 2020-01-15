package org.frc5687.infiniterecharge.robot.subsytems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Turret extends OutliersSubsystem {

    TalonSRX _turretController;


    public Turret(OutliersContainer container) {
        super(container);

        try {
            _turretController = new TalonSRX(RobotMap.CAN.TALONSRX.TURRET);
        } catch (Exception e) {

        }
    }

    @Override
    public void periodic() {

    }

    @Override
    public void updateDashboard() {
    }

}
