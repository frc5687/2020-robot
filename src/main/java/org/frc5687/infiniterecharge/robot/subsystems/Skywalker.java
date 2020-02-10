package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Skywalker extends OutliersSubsystem {
    private VictorSPX _skywalkerController;
    private OI _oi;

    public Skywalker(OutliersContainer container, OI oi) {
        super(container);
        _oi = oi;

        _skywalkerController = new VictorSPX(RobotMap.CAN.VICTORSPX.SKYWALKER);
        _skywalkerController.setNeutralMode(NeutralMode.Brake);
    }


    public void setSpeed(double speed) {
        _skywalkerController.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void updateDashboard() {

    }
}

