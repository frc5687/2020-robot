package org.frc5687.infiniterecharge.robot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc5687.infiniterecharge.robot.commands.AutoAlign;
import org.frc5687.infiniterecharge.robot.commands.AutoDrivePath;
import org.frc5687.infiniterecharge.robot.commands.AutoIntake;
import org.frc5687.infiniterecharge.robot.subsystems.*;

public class TenBallAuto extends SequentialCommandGroup {
    public TenBallAuto(DriveTrain driveTrain, Turret turret, Shooter shooter, Hood hood, Intake intake, AHRS imu) {
        addCommands(
                new ParallelDeadlineGroup(
                        new AutoDrivePath(driveTrain, imu, "Snipe", 0,false),
                        new AutoIntake(intake)
                )
                ,new WaitCommand(1)
//                , new AutoAlign(driveTrain, 50)
              ,  new AutoDrivePath(driveTrain, imu, "SnipeToShoot", 0, true)
//              ,  new AutoAlign(driveTrain, 0)
              ,  new ParallelDeadlineGroup(
                        new AutoDrivePath(driveTrain, imu, "ShootToGenerator", 0, false),
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
