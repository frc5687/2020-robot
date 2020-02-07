package org.frc5687.infiniterecharge.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.subsystems.DriveTrain;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;
    private AHRS _imu;
    private PIDController _angleController;

    private double _anglePIDOut;
    private boolean _useAnglePID;


    public Drive(DriveTrain driveTrain, OI oi, AHRS imu) {
        _driveTrain = driveTrain;
        _oi = oi;
        _imu = imu;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
//        _angleController = new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD);
//        _angleController.enableContinuousInput(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
//        _angleController.setTolerance(Constants.DriveTrain.ANGLE_TOLERANCE);
//        _useAnglePID = false;
    }

    @Override
    public void execute() {
        super.execute();
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();
//        if (wheelRotation==0 && stickSpeed != 0) {
//            _useAnglePID = true;
//            double yaw = _imu.getYaw();
//            _anglePIDOut = _angleController.calculate(yaw);
//        } else {
//            _useAnglePID = false;
//        }
//
//        if (wheelRotation==0 && _useAnglePID) {
//            metric("PID/AngleOut", _anglePIDOut);
//            metric("PID/Yaw", _imu.getYaw());
//            _driveTrain.cheesyDrive(stickSpeed, stickSpeed==0 ?  0 :_anglePIDOut, false, true);
//        } else {
            _driveTrain.cheesyDrive(stickSpeed, -wheelRotation, false, false);
//        }

    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
