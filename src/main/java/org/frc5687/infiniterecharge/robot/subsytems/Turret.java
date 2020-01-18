package org.frc5687.infiniterecharge.robot.subsytems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.DriveTurret;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Turret extends OutliersSubsystem {

    private TalonSRX _turretController;
    private Limelight _limelight;
    private OI _oi;


    public Turret(OutliersContainer container, Limelight limelight, OI oi) {
        super(container);
        _limelight = limelight;
        _oi = oi;

        try {
            debug("allocating turret motor");
            _turretController = new TalonSRX(RobotMap.CAN.TALONSRX.TURRET);
            _turretController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition,0,0);

        } catch (Exception e) {
            error("error allocating turret motors " + e.getMessage());
        }
    }

    public void setSpeed(double speed) {
        _turretController.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
        setDefaultCommand(new DriveTurret(this, _limelight, _oi));
    }

    @Override
    public void updateDashboard() {
        metric("Position", getPosition() / 100);
    }

    public double getPosition() {
        return _turretController.getSelectedSensorPosition();
    }
}
