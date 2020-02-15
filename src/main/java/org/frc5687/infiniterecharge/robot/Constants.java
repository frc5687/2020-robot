package org.frc5687.infiniterecharge.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {
    public static final int CYCLES_PER_SECOND = 50;
    public static final int  TICKS_PER_UPDATE = 10;
    public static final double METRIC_FLUSH_PERIOD = 1.0;

    public static class Intake {
        public static final boolean INTAKE_MOTOR_INVERTED = true;
        public static final double INTAKE_SPEED = 1.0;
    }

    public static class Climber {
        public static final boolean ELEVATOR_MOTOR_INVERTED = true;
        public static final boolean WINCH_MOTOR_INVERTED = true;
        public static final double ELEVATOR_EXTEND_SPEED = 0.5;
        public static final double ELEVATOR_TENSION_SPEED = 0.0;
        public static final double WINCH_RETRACT_SPEED = 1;
        public static final double WINCH_TENSION_SPEED = 0.0;
        public static final double ELEVATOR_RETRACT_SPEED = -0.50;
    }

    public static class Skywalker {
        public static final boolean SKYWALKER_MOTOR_INVERTED = true;
        public static final double DEADBAND = 0.25;
    }

    public static class DriveTrain {
        public static final double DEADBAND = 0.25;
        public static final double SPEED_SENSITIVITY = 0.9;
        public static final double ROTATION_SENSITIVITY = 0.75;

        public static final double CREEP_FACTOR = 0.25;
        public static final int CPR = 8192;
        public static final double ENCODER_CONVERSION = 6.85714286;

        public static final double WIDTH = Units.inchesToMeters(27.0);

        public static final double KS_VOLTS = 0.172;
        public static final double KV_VOLTSPR = 2.46;
        public static final double KA_VOLTSQPR = 0.355;

        public static final double RAMSETE_B = 2;
        public static final double RAMETE_ZETA = 0.7;

        public static final double KP_DRIVE_VELOCITY = 13.2;

        public static final boolean LEFT_MOTORS_INVERTED = false;
        public static final boolean RIGHT_MOTORS_INVERTED = true;

        public static final long LOCK_TIME = 80;
        public static final long DROPOUT_TIME = 100;
        public static final long SEEK_TIME = 500;

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

        public static final double SLOW_ZONE_COMP = 30;
        public static final double MEDIUM_ZONE_COMP = 70;

        public static final double SLOW_SPEED_COMP = 0.4;
        public static final double MEDIUM_SPEED_COMP = 0.6;
        public static final double kP = 0.01;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double ANGLE_TOLERANCE = 0.25;
        public static final double ROTATION_SENSITIVITY_HIGH_GEAR = .8;
        public static final double ROTATION_SENSITIVITY_LOW_GEAR = .8;
        public static final double TURNING_SENSITIVITY_HIGH_GEAR = .8;
        public static final double TURNING_SENSITIVITY_LOW_GEAR = .8;
        public static final double SPEED_LIMIT = 0.85;
    }

    public static class Turret {
        public static final double DEADBAND = 0.1;
        public static final double TOLERANCE = 2;
        public static final boolean SENSOR_PHASE_INVERTED = false;
        public static final double TICKS_TO_DEGREES = 0.08695652173913;
        public static final double MIN_DEGREES = -200;
        public static final double MAX_DEGREES = 95;
        public static final double MAX_VOLTAGE = 12.0;
        public static final int CRUISE_VELOCITY = 5000; // in ticks
        public static final int ACCELERATION = 16000; // in ticks
        public static final double ABS_OFFSET = 258;// if the turret coasts this value changes, need to find a way to set this position.


        public static class Position {
            public static double kP = 0.11;
            public static double kI = 0;
            public static double kD = 0.0016;
            public static double kF = 0;
        }

        public static class Velocity {
            public static double kP = 0.11;
            public static double kI = 0;
            public static double kD = 0.0016;
            public static double kF = 0;
        }

        public static class MotionMagic {
            public static double kP = 15;
            public static double kI = 0;
            public static double kD = 150;
            public static double kF = 4;
        }
    }

    public static class Hood {
        public static final double DEADBAND = 0.1;
        public static final double MIN_DEGREES = 0.0;
        public static final boolean SENSOR_PHASE_INVERTED = false;
        public static final double MAX_DEGREES = 360.00;
        public static final double TICKS_TO_DEGREES = 12;
        public static final double STOWED = 0;
        public static final double DISTANCE_ANGLE_CONVERSION = 0.001;
        public static final int CRUISE_VELOCITY = 5000;
        public static final int ACCELERATION = 16000;
    }

    public static class OI {
        public static final double AXIS_BUTTON_THRESHHOLD = 0.2;
        public static final long RUMBLE_MILLIS = 250;
        public static final double RUMBLE_INTENSITY = 1.0;
        public static final long RUMBLE_PULSE_TIME = 100;
        public static final int KILL_ALL = 4;
        public static final int OVERRIDE = 8;
        public static final int PANIC = 6;
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
        public static final double TARGET_HEIGHT = 92;
        public static final double LOW_TARGET_HEIGHT = 16.5;
        public static final double LIMELIGHT_HEIGHT = 8.125;

        public static final double LIMELIGHT_ANGLE = 0;
        public static final double OVERALL_LATENCY_MILLIS = 11;
    }

    public static class AutoPositions {
        public static double WIDTH_FIELD = Units.inchesToMeters(323.25);
        public static double MID_WIDTH_FIELD = WIDTH_FIELD / 2;
        public static double LENGTH_FIELD = Units.inchesToMeters(629.25);
        public static double MID_LENGTH_FIELD = LENGTH_FIELD / 2;
        public static Pose2d TARGET_POSE = new Pose2d(MID_LENGTH_FIELD,MID_LENGTH_FIELD - Units.inchesToMeters(94.66), new Rotation2d(0));
        public static Pose2d LOADING_STATION_POSE = new Pose2d(-MID_LENGTH_FIELD, 1.700911, new Rotation2d(0));

    }

    public class Spinner {
        public static final double MOTOR_PERCENT_SPEED = 0.5; // TODO: Need a real value here!
        public static final double COLOR_TOLERANCE = 0.06;
        public static final double MOTOR_SLOW_PERCENT_SPEED = 0.2; // TODO: Need a real value here!
        public static final int AUTOSPIN_SLOW_AT_WEDGES = 20;
        public static final int AUTOSPIN_STOP_AT_WEDGES = 25;
        public static final double SENSOR_SAMPLE_PERIOD_SECONDS = 0.01;
    }

    public class RotarySwitch {
        public static final double TOLERANCE = 0.02;
    }

    public class Shooter {
        public static final boolean LEFT_INVERTED = false;
        public static final boolean RIGHT_INVERTED = true;
        public static final double DEADBAND = 0.1;
        public static final double RPM_TOLERANCE = 300; //RPM
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        public static final long TIMEOUT = 100;
    }

    public class Indexer {
        public static final boolean INVERTED = false;
        public static final double ADVANCE_SPEED = 0.75; // TODO: Need a real value here!
        public static final double AGITATOR_SPEED = -1.0;
    }

    public class Auto {
        public class Drive {
            public static final double SPEED = 1.0;
            public static final double MIN_SPEED = 0.25;
            public static final double MIN_TRACK_DISTANCE = 18;
            public static final int MAX_GARBAGE = 5;
            public static final double STEER_K = 0.019;
            public static final double MAX_IMU_ANGLE = 180.0;
            public static final double MIN_IMU_ANGLE = -MAX_IMU_ANGLE;
        }
    }
}
