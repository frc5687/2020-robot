package org.frc5687.infiniterecharge.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class AutoAlign extends PIDCommand {
    public AutoAlign(DriveTrain driveTrain, double angle) {
        super(
                new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                // Close loop on heading
                driveTrain::getYaw,
                // Set reference to target
                angle,
                // Pipe output to turn robot
                output -> driveTrain.cheesyDrive(0, output, false, true),
                // Require the drive
                driveTrain);

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(Constants.DriveTrain.ANGLE_TOLERANCE);
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
    }
}
