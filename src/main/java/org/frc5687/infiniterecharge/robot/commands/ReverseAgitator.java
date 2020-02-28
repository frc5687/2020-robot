package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.Indexer;

public class ReverseAgitator extends OutliersCommand{
    private Indexer _indexer;
    public ReverseAgitator(Indexer indexer) {
        _indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        _indexer.setAgitatorSpeed(-Constants.Indexer.AGITATOR_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
