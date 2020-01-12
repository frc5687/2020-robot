package frc.robot.util;

public interface ILoggingSource {
    void error(String message);
    void warn(String message);
    void info(String message);
    void debug(String message);
}
