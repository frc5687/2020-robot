package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.Skywalker;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

public class AutoSpinToColor extends OutliersCommand {
    private Spinner _spinner;
    private OI _oi;
    private Spinner.MatchedColor _colorWeWantToSee;

    /**
     * The constructors parameters will always have the Subsystem is is trying to run
     * as Commands need to require the subsystem they are trying to use.
     */
    public AutoSpinToColor(Spinner spinner, OI oi, Skywalker skywalker) {
        _spinner = spinner;
        _oi = oi;
        addRequirements(_spinner, skywalker);
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
        Spinner.MatchedColor colorTheFieldWantsToSee = _spinner.getTargetColorFromField();
        _colorWeWantToSee = _spinner.getColorTheRobotSeesForColorTheFieldSees(colorTheFieldWantsToSee);
        _spinner.deploy(); // deploys spinner arm
        if (shouldGoLeft()) {
            _spinner.spinBackwardsSlowly();
        } else {
            _spinner.spinSlowly();
        }
    }

    @Override
    public void execute() {
        /**
         * Execute is where the code will be running continuously until isFinished() has returned true
         * execute() is run every 20ms(unless told otherwise)
         * this is where closed loop and open loop control is taken place.
         */
        if (_spinner.getColor() == _colorWeWantToSee) {
            _spinner.stop(); // stops spinner
        }
    }

    @Override
    public boolean isFinished() {
        /**
         * as the method named is stated isFinished() checks if it has ever been set to true to stop the command for continuing the loop.
         */
        return _oi.isKillAllPressed();
    }

    @Override
    public void end(boolean interrupted) {
        _spinner.stow(); // stows spinner
        _spinner.setSpeed(0);
        super.end(interrupted);
    }

    /**
     * @return Returns true if rotating the disc counterclockwise is the shortest path to the targetColor.
     */
    private boolean shouldGoLeft() {
        Spinner.MatchedColor onColor = _spinner.getColor();
        return onColor.equals(Spinner.MatchedColor.red) && _colorWeWantToSee.equals((Spinner.MatchedColor.yellow)) ||
                onColor.equals(Spinner.MatchedColor.yellow) && _colorWeWantToSee.equals((Spinner.MatchedColor.blue)) ||
                onColor.equals(Spinner.MatchedColor.blue) && _colorWeWantToSee.equals((Spinner.MatchedColor.green)) ||
                onColor.equals(Spinner.MatchedColor.green) && _colorWeWantToSee.equals((Spinner.MatchedColor.red));
    }
}
