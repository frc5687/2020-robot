package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.VictorSP;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.DigitalIR;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Indexer extends OutliersSubsystem {

    private CANSparkMax _indexerNeo;

    private VictorSPX _agitatorVictor;

    private OI _oi;

    private DigitalIR _bottomIR;
    private DigitalIR _midIR;
    private DigitalIR _topIR;

    public Indexer(OutliersContainer container) {
        super(container);

        _indexerNeo = new CANSparkMax(RobotMap.CAN.SPARKMAX.INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _indexerNeo.setInverted(Constants.Indexer.INVERTED);
        _indexerNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);

        _agitatorVictor = new VictorSPX(RobotMap.CAN.VICTORSPX.AGITATOR);

        _bottomIR = new DigitalIR(RobotMap.DIO.BOTTOM_IR);
        _midIR = new DigitalIR(RobotMap.DIO.MID_IR);
        _topIR = new DigitalIR(RobotMap.DIO.TOP_IR);

    }

    public boolean isTopTriggered() {
        return _topIR.get();
    }

    public boolean isMidTriggered() {
        return _midIR.get();
    }

    public boolean isBottomTriggered() {
        return _bottomIR.get();
    }

    public void setIndexerSpeed(double speed) {
        _indexerNeo.set(speed);
    }

    public void setAgitatorSpeed(double speed) { _agitatorVictor.set(ControlMode.PercentOutput, speed); }

    @Override
    public void updateDashboard() {
        metric("IR1", _topIR.get());
        metric("IR2", _midIR.get());
        metric("IR3", _bottomIR.get());
    }

    public boolean anyBallsDetected() {
        return _bottomIR.get() || _midIR.get() || _topIR.get();
    }
}
