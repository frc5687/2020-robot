package org.frc5687.infiniterecharge.robot.commands;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;
public class AutoDrive extends PIDCommand {

    public AutoDrive(DriveTrain driveTrain, double distance) {
        super(
                new PIDController(Constants.DriveStraight.kP, Constants.DriveStraight.kI, Constants.DriveStraight.kD),
                // Close loop on heading
                driveTrain::getDistance,
                // Set reference to target
                distance,
                // Pipe output to turn robot
                output -> driveTrain.cheesyDrive(output, 0, false, true),
                // Require the drive
                driveTrain);
        // Set the controller to be continuous (because it is  an angle controller)
        //    getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(Constants.DriveTrain.DISTANCE_TOLERANCE);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}