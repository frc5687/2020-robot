package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.DriveHood;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

import static org.frc5687.infiniterecharge.robot.Constants.Hood.MAX_ANGLE;
import static org.frc5687.infiniterecharge.robot.Constants.Hood.MIN_ANGLE;

public class Hood extends OutliersSubsystem {

    private OI _oi;
    private VictorSPX _hood;
    private DutyCycleEncoder _encoder;
    private PIDController _angleController;

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
        _angleController = new PIDController(Constants.Hood.kP, Constants.Hood.kI, Constants.Hood.kD);
        _angleController.setTolerance(Constants.Hood.TOLERANCE);
        _angleController.enableContinuousInput(MIN_ANGLE, MAX_ANGLE);
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

    public double getAngle() {
        return getPosition() * Constants.Hood.ROTATIONS_TO_DEGREES;
    }

    public void setAngle(double angle) {
        _angleController.calculate(getAngle(), angle);
    }


}
