package org.frc5687.infiniterecharge.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import java.io.IOException;

public class AutoDrivePath extends OutliersCommand {
    private Trajectory _leftTrajectory;
    private Trajectory _rightTrajectory;
    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;
    private Notifier _pathNotifier;

    private DriveTrain _driveTrain;
    private AHRS _imu;

    private double kPangle = .001;
    private double kIangle = .0001;
    private double kDangle = .001;
    private int _index = 0;
    private int _trackingThreshold;
    private String _path;
    private State _state;


    private boolean _backwards;
    private int _direction;

    public AutoDrivePath(DriveTrain driveTrain, AHRS imu, String path, int trackingSegments, boolean backwards) throws IOException {
        addRequirements(driveTrain);
        _driveTrain = driveTrain;
        _imu = imu;

        _backwards = backwards;
        _direction = backwards ? -1 : 1;

        _path = path;
        info("Loading trajectories for " + path);
        _leftTrajectory = PathfinderFRC.getTrajectory(_path + ".right");
        _rightTrajectory = PathfinderFRC.getTrajectory(_path + ".left");
        info("Left has " + _leftTrajectory.length() + " segments.");
        info("Right has " + _rightTrajectory.length() + " segments.");
        _trackingThreshold = _leftTrajectory.length() - trackingSegments;
        // logMetrics("Segment", "State", "LeftDistance", "RightDistance", "LeftSpeed","RightSpeed","Yaw","Heading","VisionHeading","DesiredHeading","HeadingDifference", "Turn","LeftOutput","RightOutput");
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MetricTracker/AutoDrivePath", true);
        super.initialize();
        _driveTrain.resetDriveEncoders();
        info("Allocating followers");

        _leftFollower = new DistanceFollower(_backwards ? _rightTrajectory : _leftTrajectory);
        _rightFollower = new DistanceFollower(_backwards ? _leftTrajectory : _rightTrajectory);

        _state = State.normal;

        error("COnfiguring follower PID");
        _leftFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);
        _rightFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);

        _pathNotifier = new Notifier(this::run);
        _pathNotifier.startPeriodic(_leftTrajectory.get(0).dt);
        _index = 0;
    }

    @Override
    public void execute() {
    }
    public void run() {
        _index++;
        metric("Segment", _index);
        metric("State",_state.name());
        double leftDistance = _driveTrain.getLeftDistance() * _direction;
        double rightDistance = _driveTrain.getRightDistance() * _direction;
        double leftSpeed = _leftFollower.calculate(leftDistance) * _direction;
        double rightSpeed = _rightFollower.calculate(rightDistance) * _direction;
        double yaw = _imu.getYaw();
        double heading = _backwards ? Pathfinder.boundHalfDegrees(yaw + 180) : yaw;
        //double heading = yaw;

        double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(_leftFollower.getHeading()));
        double headingDifference = Pathfinder.boundHalfDegrees(desiredHeading - heading);
        double turn =  Constants.AutoDrivePath.K_TURN * (1.0/80.0) * headingDifference;

/*        metric("LeftDistance",leftDistance);
        metric("RightDistance", rightDistance);
        metric("LeftSpeed",leftSpeed);
        metric("RightSpeed", rightSpeed);
        metric("Yaw", yaw);
        metric("Heading", heading);
        metric("DesiredHeading", desiredHeading);
        metric("HeadingDifference", headingDifference);
        metric("Turn", turn);
        metric("LeftOutput",leftSpeed + turn);
        metric("RightOutput", rightSpeed - turn);
*/
        _driveTrain.setPower(leftSpeed + turn, rightSpeed - turn, true);
    }

    @Override
    public boolean isFinished() {
        if (_leftFollower.isFinished() || _rightFollower.isFinished()) {
            info("AutoDrivePath finished");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _leftFollower.reset();
        _rightFollower.reset();
        if (_pathNotifier != null) { _pathNotifier.stop(); }
        info("Ending AutoDrivePath");
    }

    private enum State {
        normal,
        tracking,
        tooclose
    }
}
