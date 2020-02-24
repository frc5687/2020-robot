package org.frc5687.infiniterecharge.robot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;

public class TenBallAuto extends SequentialCommandGroup {
    public TenBallAuto(DriveTrain driveTrain, Turret turret, Shooter shooter, Hood hood, Intake intake, Indexer indexer, AHRS imu, Limelight limelight, PoseTracker poseTracker, Lights lights) {
        addCommands(
                new ParallelDeadlineGroup(
                        new AutoDrivePath(driveTrain, imu, "Snipe", 0,false),
                        new AutoIntake(intake, lights, false)
                )
              ,  new AutoDrivePath(driveTrain, imu, "SnipeToShoot", 0, true)
                ,new ParallelDeadlineGroup(
                        new AutoShoot(shooter, indexer, turret, null)
                        ,new AutoTarget(turret, shooter, hood, limelight, driveTrain, intake, poseTracker, lights,null,4300, 61, true) //TODO: Tune
                )
              ,  new ParallelDeadlineGroup(
                        new AutoDrivePath(driveTrain, imu, "ShootToGenerator", 0, false),
                        new AutoIntake(intake, lights, false)
                )
                ,new AutoDrivePath(driveTrain, imu, "HalfTrench", 0, false)

                ,  new ParallelDeadlineGroup(
                        new AutoDrivePath(driveTrain, imu, "TrenchBalls", 0, false),
                        new AutoIntake(intake, lights, true)
                )
                ,new ParallelDeadlineGroup(
                        new AutoShoot(shooter, indexer, turret, null),
                        new AutoAlign(driveTrain, 0),
                        new AutoTarget(turret, shooter, hood, limelight, driveTrain,intake, poseTracker, lights,null,4500, 63, true) //TODO: Tune
                )
        );
    }

}
