package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.DigitalIR;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Skywalker extends OutliersSubsystem {

    private DigitalIR _upIR;
    private DigitalIR _downIR;

    public Skywalker(OutliersContainer container) {
        super(container);

        _upIR = new DigitalIR(RobotMap.DIO.UP_IR);
        _downIR = new DigitalIR(RobotMap.DIO.DOWN_IR);
    }

    public boolean isUpTriggered() {
        return _upIR.get();
    }

    public boolean isDownTriggered() {
        return _downIR.get();
    }

    @Override
    public void updateDashboard() {
        metric("Up Triggered", _upIR.get());
        metric("Down Triggered", _downIR.get());
    }
}

