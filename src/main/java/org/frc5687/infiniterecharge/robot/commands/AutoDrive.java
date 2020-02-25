package org.frc5687.infiniterecharge.robot.commands;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
import org.frc5687.infiniterecharge.robot.util.Helpers;

public class AutoDrive extends PIDCommand {
    private DriveTrain _driveTrain;
    public AutoDrive(DriveTrain driveTrain, double distance, double speed) {
        super(
                new PIDController(Constants.DriveStraight.kP, Constants.DriveStraight.kI, Constants.DriveStraight.kD),
                // Close loop on heading
                driveTrain::getDistance,
                // Set reference to target
                distance,
                // Pipe output to turn robot
                output -> driveTrain.cheesyDrive(Helpers.limit(output, speed), 0, false, true),
                // Require the drive
                driveTrain);
        getController().setTolerance(Constants.DriveTrain.DISTANCE_TOLERANCE);
        _driveTrain = driveTrain;
    }
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        _driveTrain.resetDriveEncoders();
    }
}