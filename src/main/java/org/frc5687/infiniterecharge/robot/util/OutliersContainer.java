package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.infiniterecharge.robot.subsystems.OutliersSubsystem;

import java.util.LinkedList;
import java.util.List;

public abstract class OutliersContainer implements ILoggingSource, IPoseTrackable {
    private List<OutliersSubsystem> _subsystems = new LinkedList<OutliersSubsystem>();

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
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

    /***
     * Registers a subsystem for periodic actions.
     * @param subsystem
     */
    public void registerSubSystem(OutliersSubsystem subsystem) {
        if (!_subsystems.contains(subsystem)) {
            _subsystems.add(subsystem);
        }
    }

    /***
     * Unregisters a subsystem for periodic actions.
     * @param subsystem
     */
    public void unregisterSubSystem(OutliersSubsystem subsystem) {
        if (_subsystems.contains(subsystem)) {
            _subsystems.remove(subsystem);
        }
    }

    public void updateDashboard() {
        _subsystems.forEach((ss) -> ss.updateDashboard());
    }

}
