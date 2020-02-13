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
    private OI _oi;

    private Spinner _spinner;

    public Skywalker(OutliersContainer container, Spinner spinner) {
        super(container);
        if (_spinner==null) {
            throw new RuntimeException("Spinner must be allocated before passing to Skywalker constructor.");
        }
        _spinner = spinner;
    }


    public void setSpeed(double speed) {
        _spinner.setSpeed(speed);
    }

    @Override
    public void updateDashboard() {

    }
}

