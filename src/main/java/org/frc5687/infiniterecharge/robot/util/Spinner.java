package org.frc5687.infiniterecharge.robot.subsytems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import org.frc5687.infiniterecharge.robot.commands.DriveSpinner;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Spinner extends OutliersSubsystem
{
    private ColorSensorV3 _colorSensor;

    /**
     Above the constructor of our class we create our private member variables denoted with an underscore.
     These are variables that cannot be called outside of the class.
     Ex.
     private CANSparkMax _spinnerSpark;
     **/

    public Spinner(OutliersContainer container)
    {
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
    public void periodic()
    {
        /**
         * Use the space to create what command the Subsystem will be running periodically (20ms)
         * Ex.
         * setDefaultCommand(new DriveSpinner(parameters in here));
         */
        setDefaultCommand(new DriveSpinner(this));
    }

    public int getRed() //gets red channel
    {
        return _colorSensor.getRed(); //returns red channel
    }

    public int getGreen() //gets green channel
    {
        return _colorSensor.getGreen(); //returns green channel
    }

    public int getBlue() //gets blue channel
    {
        return _colorSensor.getBlue(); //returns blue channel
    }

    public int getColour() //gets most likely colour
    {
        return _colorSensor.getColor(); //gets colour
    }
    public int getIR()
    {
        return _colorSensor.getIR(); //returns sensor IR what does this do?
    }

    public int getProximity()
    {
        return _colorSensor.getProximity(); //returns sensor proximity what ever the hell that is
    }

    // TODO Fill these in based on measured results.

    public boolean isYellow()  //the following booleans I don't get
    {
        return false;
    }

    public boolean hasReset() //has the color sensor been reset
    {
        return _colorSensor.hasReset();
    }

    public boolean isRed()
    {
        return false;
    }

    public boolean isBlue()
    {
        return false;
    }

    public boolean isGreen()
    {
        return false;
    }

    public boolean colourInRange() //Is colour wheel in optimal range
    {
        int prox = getProximity(); //prox equals target proximity
        if(prox > 0 && prox < 100) //Is prox in exceptable range
        {
            return true; //Is in range
        }
        else
        {
            return false; //Is not in range
        }
    }

    public void goColour(int colour) //takes in a int which is the target colour
    {
        switch (colour)
        {
            case (1): //red
                red();
                break;

            case (2): //green
                green();
                break;

            case (3): //blue
                blue();
                break;
        }
    }

    public void red() //spins to red
    {
        boolean isRedB = false;
        int red = getRed();
        do
        {
            //motor spinny stuff
            if(red > 0 && red < 100) // is the in the red range
            {
                isRedB = true;
            }
        }while(isRedB != true); // bad number
    }

    public void green() //spins to green
    {
        boolean isGreenB = false;
        int green = getGreen();
        do
        {
            //more motor spinny stuff
            if(green > 0 && green < 100) // is the in the green range
            {
                //yup
            }
        }while(isGreenB != true);
    }

    public void blue() //spins to blue
    {
        boolean isBlueB = false;
        int blue = getBlue();
        do
        {
            if(blue > 0 && blue < 100) // is the in the blue range
            {
                //yup
            }
        }while(isBlueB != true);
    }

    @Override
    public void updateDashboard()
    {
        SmartDashboard.putNumber("Spinner/Red", getRed()); //send dash red channel values
        SmartDashboard.putNumber("Spinner/Green", getGreen()); //send dash green values
        SmartDashboard.putNumber("Spinner/Blue", getBlue()); //send dash blue values
        SmartDashboard.putNumber("Spinner/IR", getIR()); //send dash IR values
        SmartDashboard.putNumber("Spinner/Proximity", getProximity()); //send dash proximity values
    }
}