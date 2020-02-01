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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import org.frc5687.infiniterecharge.robot.commands.KillAll;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.Limelight;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;
import org.frc5687.infiniterecharge.robot.util.PDP;

import java.util.ArrayList;

public class RobotContainer extends OutliersContainer {

    private OI _oi;

    private AHRS _imu;
    private DriveTrain _driveTrain;
    private Turret _turret;
    private Shifter _shifter;
    private PDP _pdp;
    private Spinner _spinner;
    private Climber _climber;

    private Limelight _limelight;
    private Intake _intake;
    public RobotContainer(Robot robot) {

    }

    public void init() {
        // OI must be first...
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);

        _imu.zeroYaw();

        // then proxies...
        _limelight = new Limelight("limelight");


        // Then subsystems....
        if (Robot.identityMode!= Robot.IdentityMode.programming) {
            _pdp = new PDP();
            _shifter = new Shifter(this);
            _intake = new Intake(this, _oi);
            _driveTrain = new DriveTrain(this, _oi, _imu, _shifter);
            _turret = new Turret(this, _limelight, _oi);
            _spinner = new Spinner(this);
            _climber = new Climber(this, _oi);

            // Must initialize buttons AFTER subsystems are allocated...
            _oi.initializeButtons(_shifter, _driveTrain, _intake, _climber);

            // Initialize the other stuff
            _driveTrain.enableBrakeMode();
            _driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        }
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

        var interiorPoints = new ArrayList<Translation2d>();
        interiorPoints.add(new Translation2d(1,1));
        interiorPoints.add(new Translation2d(2, -1));
        Trajectory test = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                interiorPoints,
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

    @Override
    public void updateDashboard() {
        super.updateDashboard();
        _oi.updateDashboard();
    }
}
