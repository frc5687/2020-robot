/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.KillAll;
import frc.robot.subsytems.DriveTrain;
import frc.robot.subsytems.Shifter;
import frc.robot.util.*;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements ILoggingSource {

  public static IdentityMode identityMode = IdentityMode.competition;
  private RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
  private RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.warn;

  private int _updateTick = 0;

  private String _name;
  private OI _oi;

  private AHRS _imu;
  private DriveTrain _driveTrain;
//  private Shifter _shifter;

  private PDP _pdp;

  private boolean _fmsConnected;

  private Command _autoCommand;

  /**
   * This function is setRollerSpeed when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    loadConfigFromUSB();
    RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
    metric("Branch", Version.BRANCH);
    info("Starting " + this.getClass().getCanonicalName() + " from branch " + Version.BRANCH);
    info("Robot " + _name + " running in " + identityMode.toString() + " mode");

    // Periodically flushes metrics (might be good to configure enable/disable via USB config file)
    new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);

    // OI must be first...
    _oi = new OI();
    _imu = new AHRS(SPI.Port.kMXP, (byte) 100);

    _imu.zeroYaw();

    // then proxies...
    _pdp = new PDP();


    // Then subsystems....
    _driveTrain = new DriveTrain(this);
//    _shifter = new Shifter(this);
    // Must initialize buttons AFTER subsystems are allocated...
    _oi.initializeButtons(this);

    // Initialize the other stuff
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    updateDashboard();
    _oi.poll();
    update();
  }
  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    _fmsConnected =  DriverStation.getInstance().isFMSAttached();
//    _driveTrain.enableBrakeMode();
//    _autoCommand = getAutonomousCommand();

    if (_autoCommand!=null) {
      _autoCommand.schedule();
    }

  }

  public void teleopInit() {
    _fmsConnected =  DriverStation.getInstance().isFMSAttached();
    //_limelight.disableLEDs();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
//    ourPeriodic();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    ourPeriodic();
  }

  private void ourPeriodic() {
    // Example of starting a new row of metrics for all instrumented objects.
    // MetricTracker.newMetricRowAll();
    MetricTracker.newMetricRowAll();

    if (_oi.isKillAllPressed()) {
      new KillAll(this).initialize();
    }

    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    //_limelight.disableLEDs();
    RioLogger.getInstance().forceSync();
    RioLogger.getInstance().close();
//        MetricTracker.flushAll();
  }


  public void updateDashboard() {
    _updateTick++;
    if (_updateTick >= Constants.TICKS_PER_UPDATE) {
      _updateTick = 0;
      _oi.updateDashboard();
      _driveTrain.updateDashboard();
      _pdp.updateDashboard();
    }
  }


  private void loadConfigFromUSB() {    String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO
    try {
      String usbDir = "/U/"; // USB drive is mounted to /U on roboRIO
      String configFileName = usbDir + "frc5687.cfg";
      File configFile = new File(configFileName);
      FileReader reader = new FileReader(configFile);
      BufferedReader bufferedReader = new BufferedReader(reader);

      String line;
      while ((line = bufferedReader.readLine())!=null) {
        processConfigLine(line);
      }

      bufferedReader.close();
      reader.close();
    } catch (Exception e) {
      identityMode = IdentityMode.competition;
    }
  }

  private void processConfigLine(String line) {
    try {
      if (line.startsWith("#")) { return; }
      String[] a = line.split("=");
      if (a.length==2) {
        String key = a[0].trim().toLowerCase();
        String value = a[1].trim();
        switch (key) {
          case "name":
            _name = value;
            metric("name", _name);
            break;
          case "mode":
            identityMode = IdentityMode.valueOf(value.toLowerCase());
            metric("mode", identityMode.toString());
            break;
          case "fileloglevel":
            _fileLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
            metric("fileLogLevel", _fileLogLevel.toString());
            break;
          case "dsloglevel":
            _dsLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
            metric("dsLogLevel", _dsLogLevel.toString());
            break;
        }
      }
    } catch (Exception e) {

    }
  }

  private boolean _wasShocked = false;

  private void update() {

  }
  @Override
  public void error(String message) {
    RioLogger.error(this, message);
  }

  @Override
  public void warn(String message) {
    RioLogger.warn(this, message);
  }

  @Override
  public void info(String message) {
    RioLogger.info(this, message);
  }

  @Override
  public void debug(String message) {
    RioLogger.debug(this, message);
  }

  public OI getOI() {
    return _oi;
  }
  public AHRS getIMU() { return _imu; }
  public DriveTrain getDriveTrain() { return _driveTrain; }
//  public Shifter getShifter() {return  _shifter;}
  public PDP getPDP() { return _pdp; }


  public enum IdentityMode {
    competition(0),
    practice(1),
    programming(2);

    private int _value;

    IdentityMode(int value) {
      this._value = value;
    }

    public int getValue() {
      return _value;
    }
  }

  public IdentityMode getIdentityMode() {
    return identityMode;
  }

  public void metric(String name, boolean value) {
    SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
  }

  public void metric(String name, String value) {
    SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
  }

  public void metric(String name, double value) {
    SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
  }

//  public Command getAutonomousCommand() {
//    var autoVoltageConstraint =
//            new DifferentialDriveVoltageConstraint(
//                    new SimpleMotorFeedforward(Constants.DriveTrain.KS_VOLTS,
//                            Constants.DriveTrain.KV_VOLTSPR,
//                            Constants.DriveTrain.KA_VOLTSQPR),
//                    _driveTrain.getKinematics(),
//                    10
//            );
//    TrajectoryConfig config =
//            new TrajectoryConfig(Constants.DriveTrain.MAX_SPEED_MPS,
//                    Constants.DriveTrain.MAX_ACCEL_MPS)
//                    // Add kinematics to ensure max speed is actually obeyed
//                    .setKinematics(_driveTrain.getKinematics())
//                    // Apply the voltage constraint
//                    .addConstraint(autoVoltageConstraint);
////    TrajectoryConfig config = new TrajectoryConfig(Constants.DriveTrain.MAX_SPEED_MPS,Constants.DriveTrain.MAX_ACCEL_MPS).setKinematics(Constants.DriveTrain.DRIVE_KINEMATICS);
//    Trajectory test = TrajectoryGenerator.generateTrajectory(
//            new Pose2d(0,0, new Rotation2d(0)),
//            List.of(
//                    new Translation2d(1,1),
//                    new Translation2d(2,-1)
//            ),
//            new Pose2d(3, 0, new Rotation2d(0)),
//            config
//    );
//    RamseteCommand ramseteCommand = new RamseteCommand(
//            test,
//            _driveTrain::getPose,
//            new RamseteController(Constants.DriveTrain.RAMSETE_B, Constants.DriveTrain.RAMETE_ZETA),
//            new SimpleMotorFeedforward(Constants.DriveTrain.KS_VOLTS,
//                    Constants.DriveTrain.KV_VOLTSPR,
//                    Constants.DriveTrain.KA_VOLTSQPR),
//            _driveTrain.getKinematics(),
//            _driveTrain::getWheelSpeeds,
//            new PIDController(Constants.DriveTrain.KP_DRIVE_VELOCITY, 0, 0),
//            new PIDController(Constants.DriveTrain.KP_DRIVE_VELOCITY, 0, 0),
//            // RamseteCommand passes volts to the callback
//            _driveTrain::tankDriveVolts,
//            _driveTrain
//    );
//
//    return ramseteCommand.andThen(() -> _driveTrain.tankDriveVolts(0, 0));
//
//  }
}
