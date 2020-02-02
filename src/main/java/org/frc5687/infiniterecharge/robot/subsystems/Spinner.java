package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.commands.DriveSpinner;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Spinner extends OutliersSubsystem {
    private ColorSensorV3 _colorSensor;
    private TalonSRX _motorController;
    private DoubleSolenoid _solenoid;

    public void raiseArm() {
        _solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void lowerArm(){
        _solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isRaised(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kForward);
    }

    public boolean isLowered(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kReverse);
    }

    public boolean isArmOff(){
        return _solenoid.get().equals(DoubleSolenoid.Value.kOff);
    }

    public void armOff(){
        _solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public Spinner(OutliersContainer container) {
        super(container);

        try {
            debug("allocating spinner color sensor");
            I2C.Port port = I2C.Port.kOnboard;
            _colorSensor = new ColorSensorV3(port);
        } catch (Exception e) {
            error("error allocating color sensor: " + e.getMessage());
            e.printStackTrace();
        }

        try {
            debug("allocating spinner motor controller");
            _motorController = new TalonSRX(RobotMap.CAN.TALONSRX.SPINNER);
            _motorController.setNeutralMode(NeutralMode.Brake);
            // TODO: Not sure if this is really what we want, just stole from turret...
            _motorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
        } catch (Exception e) {
            error("error allocating spinner motor controller: " + e.getMessage());
            e.printStackTrace();
        }

        setDefaultCommand(new DriveSpinner(this));
    }

    // TODO Fill these in based on measured results.
    public boolean isYellow() {
        return false;
    }

    public boolean isRed() {
        return false;
    }

    public boolean isBlue() {
        return false;
    }

    public boolean isGreen() {
        return false;
    }

    public void spin() {
        _motorController.set(ControlMode.PercentOutput, Constants.Spinner.MOTOR_PERCENT_SPEED);
    }

    public void stop() {
        _motorController.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void updateDashboard() {
        metric("Spinner/Red", _colorSensor.getRed());
        metric("Spinner/Green", _colorSensor.getGreen());
        metric("Spinner/Blue", _colorSensor.getBlue());
        metric("Spinner/IR", _colorSensor.getIR());
        metric("Spinner/Proximity", _colorSensor.getProximity());
        metric("Spinner/ArmIsRaised", isRaised());
        metric("Spinner/ArmIsLowered", isLowered());
        metric("Spinner/ArmIsOff", isArmOff());
        metric("Spinner/SpinnerSpeed", _motorController.getSelectedSensorVelocity()); // units per 100ms
    }
}
