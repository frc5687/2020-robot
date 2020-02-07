package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.cuforge.libcu.Lasershark;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.DriveHood;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Hood extends OutliersSubsystem {

    private OI _oi;
    private VictorSPX _hood;
    private DutyCycleEncoder _encoder;

    private Lasershark _frontShark;
    private Lasershark _rearShark;

    public Hood(OutliersContainer container, OI oi) {
        super(container);
        _oi = oi;
        try {
            debug("Allocating hood motor");
            _hood = new VictorSPX(RobotMap.CAN.VICTORSPX.HOOD);
            _encoder = new DutyCycleEncoder(RobotMap.DIO.HOOD_ENCODER);
            _frontShark = new Lasershark(RobotMap.DIO.FRONT_SHARK);
            _rearShark = new Lasershark(RobotMap.DIO.REAR_SHARK);

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
        metric("FrontLaserDistance", getFrontDistance());
        metric("RearLaserDistance", getRearDistance());

    }

    public double getPosition() {
        return _encoder.getDistance();
    }

    public double getFrontDistance() {
        return _frontShark.getDistanceInches();
    }

    public double getRearDistance() {
        return _frontShark.getDistanceInches();
    }
}
