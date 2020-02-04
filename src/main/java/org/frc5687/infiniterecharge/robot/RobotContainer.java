package org.frc5687.infiniterecharge.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Filesystem;
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
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import org.frc5687.infiniterecharge.robot.commands.KillAll;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Intake;
import org.frc5687.infiniterecharge.robot.subsystems.Shifter;
import org.frc5687.infiniterecharge.robot.subsystems.Turret;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.subsystems.Spinner;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;
import org.frc5687.infiniterecharge.robot.util.PDP;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

public class RobotContainer extends OutliersContainer {

    private OI _oi;

    private AHRS _imu;
    private DriveTrain _driveTrain;
    private Turret _turret;
    private Shifter _shifter;
    private PDP _pdp;
    private Spinner _spinner;

    private Limelight _limelight;
    private Intake _intake;
    private Trajectory _trajectory;


    public RobotContainer(Robot robot) {

    }

    public void init() {
        // OI must be first...
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);

        _imu.zeroYaw();

        // then proxies...
        _pdp = new PDP();
        _limelight = new Limelight("limelight");


        // Then subsystems....
        _shifter = new Shifter(this);
//        _intake = new Intake(this, _oi);
        _driveTrain = new DriveTrain(this, _oi, _imu, _shifter);
        _turret = new Turret(this, _limelight, _oi);
//        _spinner = new Spinner(this);

        // Must initialize buttons AFTER subsystems are allocated...
        _oi.initializeButtons(_shifter, _driveTrain, _intake, _imu);

        // Initialize the other stuff
        try {
            String trajectoryJson = "paths/output/MidShieldGeneratorBalls.wpilib.json";
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
            _trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (Exception e) {
            error("Paths not working" + e.getMessage());
        }
        _driveTrain.enableBrakeMode();
        _driveTrain.resetOdometry(new Pose2d(0,0,new Rotation2d(0)));
    }

    public void zeroSensors() {
        _turret.zeroSensors();
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
                        _driveTrain.getDriveTrainFeedForward(),
                        _driveTrain.getKinematics(),
                        10
                );
        var interiorPoints = new ArrayList<Translation2d>();
        interiorPoints.add(new Translation2d(2,-2));
        interiorPoints.add(new Translation2d(3, -1));
        Trajectory test = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                interiorPoints,
                new Pose2d(4, 0, new Rotation2d(0)),
                _driveTrain.getDriveConfig(false)
        );
        var poseList = new ArrayList<Pose2d>();
        poseList.add(new Pose2d(0,0, new Rotation2d(0)));
        poseList.add(new Pose2d(2, -2, new Rotation2d(0)));
        poseList.add(new Pose2d(4,0, new Rotation2d(0)));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                poseList,
                _driveTrain.getDriveConfig(false)
        );

        var transform = _driveTrain.getPose().minus(_trajectory.getInitialPose());
        Trajectory trajectory2 = _trajectory.transformBy(transform);
        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory2,
                _driveTrain::getPose,
                new RamseteController(Constants.DriveTrain.RAMSETE_B, Constants.DriveTrain.RAMSETE_ZETA),
                _driveTrain.getDriveTrainFeedForward(),
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
