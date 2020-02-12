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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.*;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Shifter;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

public class RobotContainer extends OutliersContainer implements IPoseTrackable {

    private OI _oi;

    private AHRS _imu;
    private DriveTrain _driveTrain;
    private Turret _turret;
    private Indexer _indexer;
    private Shooter _shooter;
    private Shifter _shifter;
    private PDP _pdp;
    private Spinner _spinner;
    private Climber _climber;
    private Skywalker _skywalker;
    private Hood _hood;

    private Limelight _limelight;
    private Limelight _driveLimelight;
    private Intake _intake;
    private Trajectory _trajectory;
    private PoseTracker _poseTracker;

    public RobotContainer(Robot robot) {

    }

    public void init() {

        // OI must be first...
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);

        _imu.zeroYaw();

        // then proxies...
        _limelight = new Limelight("limelight");
        _driveLimelight = new Limelight("limelight-drive");


        if (Robot.identityMode!= Robot.IdentityMode.programming) {
            _pdp = new PDP();
            _shifter = new Shifter(this);
            _intake = new Intake(this, _oi);
            _driveTrain = new DriveTrain(this, _oi, _imu, _shifter);
            _turret = new Turret(this, _driveTrain, _limelight, _oi);
            _spinner = new Spinner(this);
            _climber = new Climber(this, _oi);
            _skywalker = new Skywalker(this, _oi);
            _shooter = new Shooter(this, _oi, _driveTrain);
            _indexer = new Indexer(this);
            _hood = new Hood(this, _oi);


            _poseTracker = new PoseTracker(this);
            // Must initialize buttons AFTER subsystems are allocated...

            _oi.initializeButtons(_shifter, _driveTrain, _turret, _limelight, _poseTracker, _intake, _shooter, _indexer, _spinner, _climber, _hood, _imu);

            // Initialize the other stuff
            // Initialize the other stuff
            _driveTrain.enableBrakeMode();
            _driveTrain.resetOdometry(new Pose2d(0,0,new Rotation2d(0)));

            // Now setup the default commands:
            setDefaultCommand(_hood, new DriveHood(_hood, _oi));
            setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi, _intake, _driveLimelight, _poseTracker, _imu));
            setDefaultCommand(_climber, new IdleClimber(_climber));
            setDefaultCommand(_skywalker, new DriveSkywalker(_skywalker, _oi));
            setDefaultCommand(_intake, new IntakeSpin(_intake, _oi));
            setDefaultCommand(_indexer, new IdleIndexer(_indexer, _intake, _spinner));
            setDefaultCommand(_shooter, new DriveShooter(_shooter, _oi));
            setDefaultCommand(_turret, new DriveTurret(_turret, _driveTrain, _limelight, _oi));
        }
    }

    /**
     * Helper function to wrap CommandScheduler.setDefaultCommand.  This allows us to pass nulls during initial development
     * without breaking.
     * @param subSystem
     * @param command
     */
    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem==null || command==null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    public void zeroSensors() {
        // _turret.zeroSensors();
    }

    public void periodic() {
        _oi.poll();
        if (_oi.isKillAllPressed()) {
            new KillAll(_driveTrain, _shooter, _indexer, _intake).schedule();
        }

        if (_oi.isPanicPressed()) {
            new MoveHoodToAngle(_hood, Constants.Hood.STOWED).schedule();
        }
    }

    @Override
    public Pose getPose() {
        return new RobotPose(_driveTrain.getDrivePose(), _turret.getPose());
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
        interiorPoints.add(new Translation2d(4,0));
        Trajectory test = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                interiorPoints,
                new Pose2d(5, 0, new Rotation2d(0)),
                _driveTrain.getDriveConfig(false)
        );
        var poseList = new ArrayList<Pose2d>();
        poseList.add(new Pose2d(0,0, new Rotation2d(0)));
        poseList.add(new Pose2d(2, -2, new Rotation2d(0)));
        poseList.add(new Pose2d(4,0, new Rotation2d(-10)));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                poseList,
                _driveTrain.getDriveConfig(false)
        );

        var transform = _driveTrain.getPose().minus(_trajectory.getInitialPose());
        Trajectory trajectory2 = _trajectory.transformBy(transform);
        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
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

    @Override
    public void updateDashboard() {
        super.updateDashboard();
        _oi.updateDashboard();
    }


}
