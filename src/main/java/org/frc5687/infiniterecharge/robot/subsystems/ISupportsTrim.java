package org.frc5687.infiniterecharge.robot.subsystems;

public interface ISupportsTrim {
    double getOffset();
    void setOffset(double value);
    void trim(double increment);
}
