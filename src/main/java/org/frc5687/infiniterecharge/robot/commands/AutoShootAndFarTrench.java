package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;

public class AutoShootAndFarTrench extends SequentialCommandGroup {

    public AutoShootAndFarTrench(Turret turret, Shooter shooter, Hood hood, Limelight limelight, DriveTrain driveTrain, PoseTracker poseTracker, Indexer indexer, Intake intake, Lights lights) {
        addCommands(
                new ZeroSensors(hood, turret),
                new ParallelDeadlineGroup(
                        new AutoDrive(driveTrain, 150, 0.65),
                        new AutoIntake(intake, lights, false)
                ),
            new AutoAlign(driveTrain, 0),
            new ParallelDeadlineGroup(
                    new AutoShoot(shooter, indexer, turret, null),
                    new AutoTarget(turret, shooter, hood, limelight, driveTrain,intake, poseTracker, lights,null, 4700, 65, true)
            ),
            new ZeroHoodAndTurret(hood, turret),
            new AutoAlign(driveTrain, 0),
            new ParallelDeadlineGroup(
                new AutoDrive(driveTrain, 175, 0.6),
                new AutoIntake(intake, lights, false)
            ),
            new AutoAlign(driveTrain, 0),
            new ParallelDeadlineGroup(
                    new AutoShoot(shooter, indexer, turret, null),
                    new AutoTarget(turret, shooter, hood, limelight, driveTrain,intake, poseTracker, lights,null, 5000, 70, true)
            )
        );
   }
}
