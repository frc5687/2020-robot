package org.frc5687.infiniterecharge.robot;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {
    public static final int CYCLES_PER_SECOND = 50;
    public static final int  TICKS_PER_UPDATE = 10;
    public static final double METRIC_FLUSH_PERIOD = 1.0;

    public static class Intake {
        public static final boolean INTAKE_MOTOR_INVERTED = true;
    }

    public static class Climber {
        public static final boolean CLIMBER_MOTOR_INVERTED = true;
    }

    public static class DriveTrain {
        public static final double DEADBAND = 0.25;
        public static final double SPEED_SENSITIVITY = 0.9;
        public static final double ROTATION_SENSITIVITY = 0.75;

        public static final double CREEP_FACTOR = 0.25;
        public static final double LEFT_DISTANCE_PER_PULSE = 0.0286206896551724;
        public static final double RIGHT_DISTANCE_PER_PULSE = 0.0286206896551724;

        public static final double WIDTH = Units.inchesToMeters(27.0);

        public static final double KS_VOLTS = 0.172;
        public static final double KV_VOLTSPR = 2.46;
        public static final double KA_VOLTSQPR = 0.355;

        public static final double RAMSETE_B = 2;
        public static final double RAMETE_ZETA = 0.7;

        public static final double KP_DRIVE_VELOCITY = 13.2;

        public static final boolean LEFT_MOTORS_INVERTED = true;
        public static final boolean RIGHT_MOTORS_INVERTED = false;

        public static final double MAX_SPEED_IPS = 156.0;
        public static final double MAX_SPEED_MPS = Units.inchesToMeters(MAX_SPEED_IPS);
        public static final double CAP_SPEED_IPS = .8 * MAX_SPEED_IPS;
        public static final double MAX_ACCELERATION_IPSS = CAP_SPEED_IPS / 2;
        public static final double MAX_ACCEL_MPS = Units.inchesToMeters(MAX_ACCELERATION_IPSS);
        public static final double MAX_JERK_IPSSS = CAP_SPEED_IPS;
        public static final double RAMP_RATE = 0.125;
        public static final int STALL_CURRENT_LIMIT = 50;
        public static final int FREE_CURRENT_LIMIT = 60;
        public static final double SECONDARY_LIMIT = 90;
    }

    public static class Turret {
        public static final double DEADBAND = 0.1;
        public static final double TOLERANCE = 2;
        public static final boolean SENSOR_PHASE_INVERTED = false;
        public static final double TICKS_TO_DEGREES = 0.08695652173913;
    }

    public static class Hood {
        public static final double DEADBAND = 0.1;
    }

    public static class OI {
        public static final double AXIS_BUTTON_THRESHHOLD = 0.2;
        public static final long RUMBLE_MILLIS = 250;
        public static final double RUMBLE_INTENSITY = 1.0;
        public static final long RUMBLE_PULSE_TIME = 100;
        public static final int KILL_ALL = 4;
        public static final int OVERRIDE = 8;
    }

    public static class Shifter {
        public static final long STOP_MOTOR_TIME = 60;
        public static final long SHIFT_TIME = 60;

        public static final double SHIFT_UP_THRESHOLD = 50; // in inches per second graTODO tune
        public static final double SHIFT_DOWN_THRESHOLD = 40; // in inches per second TODO tune

        public static final long AUTO_WAIT_PERIOD = 500;
        public static final long MANUAL_WAIT_PERIOD = 3000;
    }
    public class Limelight {
        public static final double TARGET_HEIGHT = 29;
        public static final double LIMELIGHT_HEIGHT = 41.5;
        public static final double LIMELIGHT_ANGLE = 20;
        public static final double OVERALL_LATENCY_MILLIS = 11;
    }

    public class Spinner {
        public static final double MOTOR_PERCENT_SPEED = 0.5; // TODO: Need a real value here!
        public static final double COLOR_TOLERANCE = 0.06;
    }

    public class RotarySwitch {
        public static final double TOLERANCE = 0.02;
    }

    public class Shooter {
        public static final boolean LEFT_INVERTED = false;
        public static final boolean RIGHT_INVERTED = true;
        public static final double DEADBAND = 0.1;
    }

    public class Indexer {
        public static final boolean INVERTED = false;
        public static final double ADVANCE_SPEED = 0.75; // TODO: Need a real value here!
    }
}
