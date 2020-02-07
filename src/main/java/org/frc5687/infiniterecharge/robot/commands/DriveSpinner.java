package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class DriveSpinner extends OutliersCommand {

    /**
     * Similar to Subsystems this is where we place out private member variables.
     */

    private Spinner _spinner;
    private OI _oi;

    /**
     * The constructors parameters will always have the Subsystem is is trying to run
     * as Commands need to require the subsystem they are trying to use.
     */
    public DriveSpinner(Spinner spinner, OI oi) {
        _spinner = spinner;
        _oi = oi;
        addRequirements(_spinner);
    }

    @Override
    public void initialize() {
        /**
         * Initialize is what the command does on its first loop
         * say that we want the spinner wheel to be in break mode, in initialize we will stay to set the spinner motor to break mode.
         * Ex.
         * _spinner.enableBreakMode(); <---- this method need to be create in the Subsystem class.
         */
        super.initialize();
    }

    @Override
    public void execute() {
        /**
         * Execute is where the code will be running continuously until isFinished() has returned true
         * execute() is run every 20ms(unless told otherwise)
         * this is where closed loop and open loop control is taken place.
         */
        super.execute();
        double speed = _oi.getSpinnerSpeed();
        _spinner.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        /**
         * as the method named is stated isFinished() checks if it has ever been set to true to stop the command for continuing the loop.
         */
        return false;
    }
}
