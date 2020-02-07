package org.frc5687.infiniterecharge.robot;

import org.frc5687.infiniterecharge.robot.util.BasicPose;
import org.frc5687.infiniterecharge.robot.util.Pose;
import org.frc5687.infiniterecharge.robot.util.TurretPose;

public class RobotPose extends Pose {
    private TurretPose _turretPose;
    private BasicPose _drivePose;

    public RobotPose(BasicPose drivePose, TurretPose turretPose) {
        _turretPose = turretPose;
        _drivePose = drivePose;
    }

    public TurretPose getTurretPose() { return _turretPose; }

    public BasicPose getDrivePose() { return _drivePose; }
}
