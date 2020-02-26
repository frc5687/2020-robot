package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;

public class AutoShootAndNearTrench extends SequentialCommandGroup {

    public AutoShootAndNearTrench(Turret turret, Shooter shooter, Hood hood, Limelight limelight, DriveTrain driveTrain, PoseTracker poseTracker, Indexer indexer, Intake intake, Lights lights) {
        addCommands(
                new ZeroSensors(hood, turret),
            new ParallelDeadlineGroup(
                    new AutoShoot(shooter, indexer, turret, null),
                    new AutoTarget(turret, shooter, hood, limelight, driveTrain,intake, poseTracker, lights, null, 3500, 50, true)
            ),
            new ParallelDeadlineGroup(
                new AutoDrive(driveTrain, 198, .8),
                new AutoIntake(intake, lights, true)
            ),
            new ParallelDeadlineGroup(
                    new AutoShoot(shooter, indexer, turret, null),
                    new AutoTarget(turret, shooter, hood, limelight, driveTrain,intake, poseTracker, lights,null,5000, 68.5, true)
            )
        );
   }
}
