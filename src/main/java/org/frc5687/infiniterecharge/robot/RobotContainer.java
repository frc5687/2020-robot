package org.frc5687.infiniterecharge.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import org.frc5687.infiniterecharge.robot.commands.KillAll;
import org.frc5687.infiniterecharge.robot.subsytems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsytems.Shifter;
import org.frc5687.infiniterecharge.robot.subsytems.Shooter;
import org.frc5687.infiniterecharge.robot.util.MetricTracker;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;
import org.frc5687.infiniterecharge.robot.util.PDP;

import java.util.List;

public class RobotContainer extends OutliersContainer {

    private OI _oi;

    private AHRS _imu;
    private DriveTrain _driveTrain;
    private Shooter _shooter;
    private Shifter _shifter;
    private PDP _pdp;

    public RobotContainer(Robot robot) {

    }

    public void init() {
        // OI must be first...
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);

        _imu.zeroYaw();

        // then proxies...
        _pdp = new PDP();


        // Then subsystems....
        _shifter = new Shifter(this);
        _driveTrain = new DriveTrain(this, _oi, _imu, _shifter);
        _shooter = new Shooter(this, _oi);

        // Must initialize buttons AFTER subsystems are allocated...
        _oi.initializeButtons(_shifter, _driveTrain);

        // Initialize the other stuff

    }


    public void periodic() {
        _oi.poll();
        if (_oi.isKillAllPressed()) {
            new KillAll(_driveTrain).schedule();
        }
    }

    public Command getAutonomousCommand() {
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(Constants.DriveTrain.KS_VOLTS,
                                Constants.DriveTrain.KV_VOLTSPR,
                                Constants.DriveTrain.KA_VOLTSQPR),
                        _driveTrain.getKinematics(),
                        10
                );
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.DriveTrain.MAX_SPEED_MPS,
                        Constants.DriveTrain.MAX_ACCEL_MPS)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(_driveTrain.getKinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        Trajectory test = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, new Rotation2d(0)),
                config
        );
        RamseteCommand ramseteCommand = new RamseteCommand(
                test,
                _driveTrain::getPose,
                new RamseteController(Constants.DriveTrain.RAMSETE_B, Constants.DriveTrain.RAMETE_ZETA),
                new SimpleMotorFeedforward(Constants.DriveTrain.KS_VOLTS,
                        Constants.DriveTrain.KV_VOLTSPR,
                        Constants.DriveTrain.KA_VOLTSQPR),
                _driveTrain.getKinematics(),
                _driveTrain::getWheelSpeeds,
                new PIDController(Constants.DriveTrain.KP_DRIVE_VELOCITY, 0, 0),
                new PIDController(Constants.DriveTrain.KP_DRIVE_VELOCITY, 0, 0),
                // RamseteCommand passes volts to the callback
                _driveTrain::tankDriveVolts,
                _driveTrain
        );

        return ramseteCommand.andThen(() -> _driveTrain.tankDriveVolts(0, 0));

    }

}
