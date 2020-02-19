package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;

import java.lang.module.FindException;

public class AutoShootAndGo extends SequentialCommandGroup {

    public AutoShootAndGo(Turret turret, Shooter shooter, Hood hood, Limelight limelight, DriveTrain driveTrain, PoseTracker poseTracker, Indexer indexer, Lights lights) {
        addCommands(
            new ParallelDeadlineGroup(
                    new AutoShoot(shooter, indexer, turret, null, 0),
                    new AutoTarget(turret, shooter, hood, limelight, driveTrain, poseTracker, lights,null,5000, 60)),
            new AutoDrive(driveTrain, 36)
        );
   }

}
