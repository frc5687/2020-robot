package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.DriveHood;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Hood extends OutliersSubsystem {

    private OI _oi;
    private VictorSPX _hood;
    private DutyCycleEncoder _encoder;

    public Hood(OutliersContainer container, OI oi) {
        super(container);
        _oi = oi;
        try {
            debug("Allocating hood motor");
            _hood = new VictorSPX(RobotMap.CAN.VICTORSPX.HOOD);
            _encoder = new DutyCycleEncoder(RobotMap.DIO.HOOD_ENCODER);
        } catch (Exception e) {
            error("Exception allocating hood motor" + e.getMessage());
        }
    }

    public void setSpeed(double speed) {
        _hood.set(ControlMode.PercentOutput, speed);
    }


    @Override
    public void updateDashboard() {
        metric("Position", getPosition());
    }

    public double getPosition() {
        return _encoder.getDistance();
    }


}
