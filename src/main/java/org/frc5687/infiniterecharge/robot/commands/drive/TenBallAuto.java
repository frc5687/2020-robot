package org.frc5687.infiniterecharge.robot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.commands.AutoAlign;
import org.frc5687.infiniterecharge.robot.commands.AutoDrivePath;
import org.frc5687.infiniterecharge.robot.commands.AutoIntake;
import org.frc5687.infiniterecharge.robot.subsystems.*;

public class TenBallAuto extends SequentialCommandGroup {
    public TenBallAuto(DriveTrain driveTrain, Turret turret, Shooter shooter, Hood hood, Intake intake, AHRS imu) {
        addCommands(
                new AutoDrivePath(driveTrain, imu, "Snipe", 0,false),
                new AutoAlign(driveTrain, 30),
                new AutoDrivePath(driveTrain, imu, "SnipeToShoot", 0, true),
                new AutoAlign(driveTrain, 0),
                new AutoDrivePath(driveTrain, imu, "ShootToGenerator", 0, false),
                new AutoAlign(driveTrain, -100),
                new AutoDrivePath(driveTrain, imu, "ShootToGenerator", 0, false)
        );
    }

}
