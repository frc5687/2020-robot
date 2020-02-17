package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Indexer;
import org.frc5687.infiniterecharge.robot.subsystems.Intake;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;

/***
 * This command acts as a giant state machine to load the hopper/indexer with balls.  The basic rules are:
 * * If there's a ball at position 1, don't run!
 *  * If there is a ball in position 2 but not position 3, don't run
 *  * If there are balls at position 3 and position 2, advance until there's a ball at position 1
 *  * If there's a ball at position 3, advance until it reaches position 2
 *  * If no balls are detected in any positions:
 *  *   If the intake is running, or there are balls detected at all, run until a ball reaches position 2
 */
public class IdleIndexer extends OutliersCommand {
    private Indexer _indexer;
    private Intake _intake;

    public IdleIndexer(Indexer indexer, Intake intake) {
        super();
        _indexer = indexer;
        _intake = intake;
        addRequirements(_indexer);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _indexer.setIndexerSpeed(0);
    }

    @Override
    public void execute() {
        super.execute();
        double speed = 0;
        // State machine logic below:
        if (_indexer.isTopTriggered()) {
            metric("State", "Top Triggered, Stopping");
            speed = 0;
        } else if (_indexer.isMidTriggered()) {
            if (_indexer.isBottomTriggered()) {
                metric("State", "Middle and Bottom Triggered, Advancing");
                speed = Constants.Indexer.ADVANCE_SPEED;
            } else {
                metric("State", "ToJust Middle Triggered, Stopping");
                speed = 0;
            }
        } else if (_indexer.isBottomTriggered()) {
            metric("State", "Just Bottom Triggered, Stopping");
            speed = Constants.Indexer.ADVANCE_SPEED;
        } else if (_intake.isRunning() || _indexer.anyBallsDetected()) {
            metric("State", "Intaking, Advancing");
            speed = Constants.Indexer.ADVANCE_SPEED;
        } else {
            metric("State", "Intaking, Advancing");
            speed = Constants.Indexer.ADVANCE_SPEED;
        }
        _indexer.setIndexerSpeed(speed);

        _indexer.setAgitatorSpeed(Constants.Indexer.AGITATOR_SPEED);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
