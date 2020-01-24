package org.frc5687.infiniterecharge.robot.subsytems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import org.frc5687.infiniterecharge.robot.commands.DriveSpinner;
import org.frc5687.infiniterecharge.robot.subsystems.OutliersSubsystem;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Spinner extends OutliersSubsystem {
    private ColorSensorV3 _colorSensor;

    /**
        Above the constructor of our class we create our private member variables denoted with an underscore.
        These are variables that cannot be called outside of the class.
        Ex.
        private CANSparkMax _spinnerSpark;
     **/

    public Spinner(OutliersContainer container) {
        super(container);
        /**
         * Inside the constructor is where we instantiate our objects and variable.
         * Different objects requires different parameters, documentation of the object provides the order and what parameters are needed.
         * Ex.
         * _spinnerSpark = new CANSparkMax(CAN ID, TYPE OF MOTOR);
         * CAN ID's are placed in RobotMap so replace with RobotMap.CAN.SPARKMAX.SPINNER
         * TYPE OF MOTOR: we use brushless so CANSparkMaxLowLevel.MotorType.kBrushless <--- this is provided my RevRobotics vendor library.
         */

        // TODO(mike) do we need to move this to RobotMap? I think we only have one i2c bus?
        I2C.Port port = I2C.Port.kOnboard;
        _colorSensor =  new ColorSensorV3(port);
    }

    @Override
    public void periodic() {
        /**
         * Use the space to create what command the Subsystem will be running periodically (20ms)
         * Ex.
         * setDefaultCommand(new DriveSpinner(parameters in here));
         */
        setDefaultCommand(new DriveSpinner(this));
    }

    public int getRed() {
        return _colorSensor.getRed();
    }
    public int getGreen() {
        return _colorSensor.getGreen();
    }
    public int getBlue() {
        return _colorSensor.getBlue();
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

    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Spinner/Red", getRed());
        SmartDashboard.putNumber("Spinner/Green", getGreen());
        SmartDashboard.putNumber("Spinner/Blue", getBlue());
        SmartDashboard.putNumber("Spinner/IR", _colorSensor.getIR());
        SmartDashboard.putNumber("Spinner/Proximity", _colorSensor.getProximity());
    }
}
