package org.frc5687.infiniterecharge.robot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc5687.infiniterecharge.robot.commands.AutoAlign;
import org.frc5687.infiniterecharge.robot.commands.AutoDrivePath;
import org.frc5687.infiniterecharge.robot.commands.AutoIntake;
import org.frc5687.infiniterecharge.robot.subsystems.*;

public class EightBallAuto extends SequentialCommandGroup {
    public EightBallAuto(DriveTrain driveTrain, Turret turret, Shooter shooter, Hood hood, Intake intake, AHRS imu) {
        addCommands(
                new ParallelDeadlineGroup(
                        new AutoDrivePath(driveTrain, imu, "StartingToGenerator", 0,false),
                        new AutoIntake(intake)
                )
//                , new AutoAlign(driveTrain, 70)
                ,  new AutoDrivePath(driveTrain, imu, "HalfTrench", 0, true)
              ,  new AutoAlign(driveTrain, 0)
                ,  new ParallelDeadlineGroup(
                        new AutoDrivePath(driveTrain, imu, "TrenchBalls", 0, false),
                        new AutoIntake(intake)
                )
//              ,  new AutoAlign(driveTrain, -100)
//              ,  new ParallelDeadlineGroup(
//                        new AutoDrivePath(driveTrain, imu, "BallsToTrench", 0, false),
//                        new AutoIntake(intake)
//                )
        );
    }
}
