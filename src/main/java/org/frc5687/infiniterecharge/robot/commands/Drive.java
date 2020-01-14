package org.frc5687.infiniterecharge.robot.commands;

import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsytems.DriveTrain;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        addRequirements(_driveTrain);
        logMetrics("X", "Y");
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MetricTracker/Drive", true);
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();
        _driveTrain.cheesyDrive(stickSpeed, wheelRotation, false, true);
        metric("Pose", _driveTrain.getPose().toString());
        metric("X", _driveTrain.getPose().getTranslation().getX());
        metric("Y", _driveTrain.getPose().getTranslation().getY());
        metric("Heading", _driveTrain.getPose().getRotation().getDegrees());
    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
