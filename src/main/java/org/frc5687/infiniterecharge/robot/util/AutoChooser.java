package org.frc5687.infiniterecharge.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.infiniterecharge.robot.Constants;
import org.frc5687.infiniterecharge.robot.RobotMap;

/**
 * Created by Ben Bernard on 2/2/2018.
 */
public class AutoChooser extends OutliersProxy {
    private RotarySwitch _modeSwitch;

    public AutoChooser(OutliersContainer.IdentityMode identityMode) {
        _modeSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH,  Constants.RotarySwitch.TOLERANCE, 0.07692, 0.15384, 0.23076, 0.30768, 0.3846, 0.46152, 0.53844, 0.61536, 0.69228, 0.7692, 0.84612, 0.92304);
        // _modeSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH, Constants.RotarySwitch.TOLERANCE, .09, .17, .23, .31, .5, .59, .68, .75, .82, .91, .96);
    }


    public Mode getSelectedMode(){
        int raw = _modeSwitch.get();
        if (raw >= Mode.values().length) { raw = 0; }
        try {
            return Mode.values()[raw];
        } catch(Exception e){
                return Mode.StayPut;
        }
    }


    public void updateDashboard(){
        metric("Label/Mode", getSelectedMode().getLabel());
        metric("Raw/Mode", _modeSwitch.getRaw());
        metric("Numeric/Mode", _modeSwitch.get());
  }

    public enum Mode {
        StayPut(0, "Stay Put"),
        ShootAndGo(1, "Shoot and Cross"),
        ShootAndNearTrench(2, "Shoot and Near Trench"),
        ShootAndFarTrench(3, "Shoot and Far Trench"),
        Generator2NearTrench(4, "Generator 2 and Near Trench"),
        Generator2FarTrench(5, "Generator 2 and Far Trench"),
        SnipeAndNearTrench(6, "Snipe and Near Trench"),
        SnipeAndFarTrench(7, "Snipe and Far Trench")
        ;

        private String _label;
        private int _value;

        Mode(int value, String label) {
            _value = value;
            _label = label;
        }

        public int getValue() { return _value; }
        public String getLabel() { return _label; }

    }
}
