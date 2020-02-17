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
            new ParallelDeadlineGroup(
                    new AutoShoot(shooter, indexer, turret, null, 0),
                    new AutoTarget(turret, shooter, hood, limelight, driveTrain, poseTracker, lights,null)
            ),
            new MoveHoodToAngle(hood, Constants.Hood.MIN_DEGREES),
            new ParallelDeadlineGroup(
                new AutoDrive(driveTrain, 250),
                new AutoIntake(intake)
            ),
            new ParallelDeadlineGroup(
                    new AutoShoot(shooter, indexer, turret, null, 0),
                    new AutoTarget(turret, shooter, hood, limelight, driveTrain, poseTracker, lights,null)
            )
        );
   }
}
