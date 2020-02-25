package org.frc5687.infiniterecharge.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.infiniterecharge.robot.commands.*;
import org.frc5687.infiniterecharge.robot.commands.drive.EightBallAuto;
import org.frc5687.infiniterecharge.robot.subsystems.*;
import org.frc5687.infiniterecharge.robot.util.*;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.subsystems.Shifter;
import org.frc5687.infiniterecharge.robot.subsystems.Shooter;

public class RobotContainer extends OutliersContainer implements IPoseTrackable {

    private OI _oi;

    private AutoChooser _autoChooser;

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

    private Lights _lights;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
    }
    public void init() {

        // OI must be first...
        _oi = new OI();
        _autoChooser = new AutoChooser(getIdentityMode());
        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);

        _imu.zeroYaw();

        // then proxies...
        _limelight = new Limelight("limelight");
        _driveLimelight = new Limelight("limelight-drive");

        _limelight.setPipeline(Limelight.Pipeline.Wide);


        // Then subsystems....
        if (Robot._identityMode != IdentityMode.programming) {
            _pdp = new PDP();
            _shifter = new Shifter(this);
            _intake = new Intake(this, _oi);
            _driveTrain = new DriveTrain(this, _oi, _imu, _shifter, _driveLimelight);
            _turret = new Turret(this, _driveTrain, _hood, _limelight, _oi);
            _spinner = new Spinner(this);
            _climber = new Climber(this, _oi);
            _skywalker = new Skywalker(this);
            _shooter = new Shooter(this, _oi, _driveTrain);
            _indexer = new Indexer(this);
            _hood = new Hood(this,_limelight, _oi);


            _poseTracker = new PoseTracker(this);

            _lights = new Lights(this, _oi);

            // Must initialize buttons AFTER subsystems are allocated...

            _oi.initializeButtons(_shifter, _driveTrain, _turret, _limelight, _poseTracker, _intake, _shooter, _indexer, _spinner, _climber, _hood, _skywalker, _lights, _imu);

            // Initialize the other stuff
            _driveTrain.enableBrakeMode();
//            _driveTrain.resetOdometry(Constants.AutoPositions.EIGHT_BALL_STARING);
            _driveTrain.resetOdometry(Constants.AutoPositions.TRENCH_STARTING);

            // Now setup the default commands:
            setDefaultCommand(_hood, new DriveHood(_hood, _oi));
            setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi, _intake, _driveLimelight, _poseTracker, _imu));
            setDefaultCommand(_climber, new IdleClimber(_climber));
             setDefaultCommand(_skywalker, new DriveSkywalker(_skywalker, _spinner, _oi));
            setDefaultCommand(_intake, new IntakeSpin(_intake, _oi));
            setDefaultCommand(_indexer, new IdleIndexer(_indexer, _intake, _lights));
            setDefaultCommand(_shooter, new DriveShooter(_shooter, _oi));
//            setDefaultCommand(_turret, new AutoTurretTracking(_turret, _driveTrain, _limelight, _oi,  _poseTracker));
            _limelight.enableLEDs();
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
         _turret.zeroSensors();
    }

    public void periodic() {
        _oi.poll();
        if (_oi.isKillAllPressed()) {
            new KillAll(_driveTrain, _shooter, _indexer, _intake, _turret, _hood).schedule();
            _indexer.stopAgitator();
        }
    }

    public void disabledPeriodic() {
        if (_indexer!=null) {
            // _indexer.abortAgitator();
        }
    };

    @Override
    public void disabledInit() {
        _indexer.stopAgitator();
    };

    @Override
    public void teleopInit() {
        _indexer.startAgitator();
    };

    @Override
    public void autonomousInit() {
        _indexer.startAgitator();
    };

    @Override
    public Pose getPose() {
        return new RobotPose(_driveTrain.getDrivePose(), _turret.getPose());
    }

    public Command getAutonomousCommand() {


        AutoChooser.Mode autoMode = _autoChooser.getSelectedMode();

        switch (autoMode) {
            case ShootAndGo:
                return wrapCommand(new AutoShootAndGo(_turret, _shooter, _hood, _limelight, _driveTrain, _intake, _poseTracker, _indexer, _lights));
            case ShootAndNearTrench:
                return wrapCommand(new AutoShootAndNearTrench(_turret, _shooter, _hood, _limelight, _driveTrain, _poseTracker, _indexer, _intake, _lights));
            case ShootAndFarTrench:
                return wrapCommand(new AutoShootAndFarTrench(_turret, _shooter, _hood, _limelight, _driveTrain, _poseTracker, _indexer, _intake, _lights));
            case Generator2NearTrench:
                return wrapCommand(new EightBallAuto(_driveTrain, _turret, _shooter,_hood,_intake, _imu, _indexer,_lights, _limelight, _poseTracker));
            default:
                return new SequentialCommandGroup(
                        new ZeroSensors(_hood, _turret),
                        new AutoShootAndGo(_turret, _shooter, _hood, _limelight, _driveTrain, _intake, _poseTracker, _indexer, _lights)
//                        new EightBallAuto(_driveTrain, _turret, _shooter,_hood,_intake, _imu, _indexer,_lights, _limelight, _poseTracker)
                );
        }
    }

    private Command wrapCommand(Command command) {
        return new SequentialCommandGroup(
                new ZeroSensors(_hood, _turret),
                command
        );
    }

    @Override
    public void updateDashboard() {
        super.updateDashboard();
        _oi.updateDashboard();
        _autoChooser.updateDashboard();
    }


}
