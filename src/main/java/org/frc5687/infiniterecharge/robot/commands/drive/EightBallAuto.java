package org.frc5687.infiniterecharge.robot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.PoseTracker;

public class EightBallAuto extends SequentialCommandGroup {
    public EightBallAuto(DriveTrain driveTrain, Turret turret, Shooter shooter, Hood hood, Intake intake, AHRS imu, Indexer indexer, Lights lights, Limelight limelight, PoseTracker poseTracker) {
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new AutoDrivePath(driveTrain, imu, "StartingToGenerator", 0,false),
                            new AutoPause(250, driveTrain)),
                        new AutoIntake(intake, lights, false)
                )
                , new AutoDrivePath(driveTrain, imu, "HalfTrench", 0, true)
                ,  new AutoAlign(driveTrain, 0)
                , new SetPose(driveTrain, Constants.AutoPositions.TRENCH_EDGE)
                ,new AutoTurretSetpoint(turret, -10)
                ,new ParallelDeadlineGroup(
                        new AutoShoot(shooter, indexer, turret, null)
                        ,new AutoTarget(turret, shooter, hood, limelight, driveTrain,intake, poseTracker, lights,null,4700, 62.5, true) //TODO: Tune
                )
                ,  new ParallelDeadlineGroup(
                        new AutoDrivePath(driveTrain, imu, "TrenchBalls", 0, false),
                        new AutoIntake(intake, lights, true)
                )
                ,new ParallelDeadlineGroup(
                    new AutoShoot(shooter, indexer, turret, null),
                        new AutoAlign(driveTrain, 0),
                        new AutoTarget(turret, shooter, hood, limelight, driveTrain,intake, poseTracker, lights,null,5000, 67.5, true) //TODO: Tune
                )
        );
    }
}
