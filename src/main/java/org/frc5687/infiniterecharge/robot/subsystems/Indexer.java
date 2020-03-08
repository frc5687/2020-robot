package org.frc5687.infiniterecharge.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.OI;
import org.frc5687.infiniterecharge.robot.RobotMap;
import org.frc5687.infiniterecharge.robot.util.DigitalIR;
import org.frc5687.infiniterecharge.robot.util.OutliersContainer;

public class Indexer extends OutliersSubsystem {

    private CANSparkMax _indexerNeo;
    private VictorSPX _agitator;

    private Servo _agitatorServo1;
    private Servo _agitatorServo2;
    private Servo _agitatorServo3;
    private Servo _agitatorServo4;
    private Servo _agitatorServo5;

    private boolean _abort;

    private OI _oi;

    private DigitalIR _bottomIR;
    private DigitalIR _midIR;
    private DigitalIR _topIR;

    public Indexer(OutliersContainer container) {
        super(container);

        _indexerNeo = new CANSparkMax(RobotMap.CAN.SPARKMAX.INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _indexerNeo.setInverted(Constants.Indexer.INVERTED);
        _indexerNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _agitator = new VictorSPX(RobotMap.CAN.VICTORSPX.AGITATOR);
        _agitator.setInverted(false);


        _agitatorServo1 = new Servo(RobotMap.PWM.AGITATOR1);
        _agitatorServo2 = new Servo(RobotMap.PWM.AGITATOR2);
        _agitatorServo3 = new Servo(RobotMap.PWM.AGITATOR3);
        _agitatorServo4 = new Servo(RobotMap.PWM.AGITATOR4);
        _agitatorServo5 = new Servo(RobotMap.PWM.AGITATOR5);
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

    public void stopAgitator() {
        _abort = true;
    }

    public void startAgitator() {
        _abort =false;
    }

    public void setAgitatorSpeed(double speed) {
        _agitator.set(ControlMode.PercentOutput, speed);
    }


    @Override
    public void periodic() {
        setAgitatorSpeed(Constants.Indexer.AGITATOR_SPEED);
        if (_abort) {
            _agitatorServo1.set(Constants.Indexer.SERVO_STOPPED);
            _agitatorServo2.set(Constants.Indexer.SERVO_STOPPED);
            _agitatorServo3.set(Constants.Indexer.SERVO_STOPPED);
            _agitatorServo4.set(Constants.Indexer.SERVO_STOPPED);
            _agitatorServo5.set(Constants.Indexer.SERVO_STOPPED);
        } else {
            _agitatorServo1.set(0.00);
            _agitatorServo2.set(0.00);
            _agitatorServo3.set(0.00);
            _agitatorServo4.set(0.00);
            _agitatorServo5.set(0.00);
        }
    }

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
