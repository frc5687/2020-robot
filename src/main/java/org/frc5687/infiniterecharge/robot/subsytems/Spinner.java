package org.frc5687.infiniterecharge.robot.subsytems;

import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Spinner extends OutliersSubsystem {
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
    }

    @Override
    public void periodic() {
        /**
         * Use the space to create what command the Subsystem will be running periodically (20ms)
         * Ex.
         * setDefaultCommand(new DriveSpinner(parameters in here));
         */
    }

    @Override
    public void updateDashboard() {

    }
}
