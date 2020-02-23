package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class SetPose extends OutliersCommand{
    private Pose2d _pose;
    private DriveTrain _driveTrain;

    public SetPose(DriveTrain driveTrain, Pose2d pose) {
        _driveTrain = driveTrain;
        _pose = pose;
    }

    @Override
    public void initialize() {
        _driveTrain.resetOdometry(_pose);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
