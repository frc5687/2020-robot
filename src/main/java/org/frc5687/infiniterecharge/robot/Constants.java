package org.frc5687.infiniterecharge.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {
    public static final int  TICKS_PER_UPDATE = 10;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final double UPDATE_PERIOD = 0.02;
    public static class Intake {
        public static final boolean INTAKE_MOTOR_INVERTED = true;
        public static final double INTAKE_SPEED = 1.0;

    }
    public static class Climber {
        public static final boolean ELEVATOR_MOTOR_INVERTED = false;
        public static final boolean WINCH_MOTOR_INVERTED = true;
        public static final double ELEVATOR_EXTEND_SPEED = 1.0;
        public static final double ELEVATOR_TENSION_SPEED = 0.0;
        public static final double WINCH_RETRACT_SPEED = 1;
        public static final double WINCH_TENSION_SPEED = 0.0;
        public static final double ELEVATOR_RETRACT_SPEED = -0.75;
        public static final double NEAR_BOTTOM = 30.0;
        public static final double NEAR_TOP = 175.0;
        public static final double ELEVATOR_EXTEND_SPEED_SLOW = 0.2;
        public static final double ELEVATOR_RETRACT_SPEED_SLOW = -0.20;
        public static final double EXTENDED = 60.0;
    }

    public static class Skywalker {
        public static final boolean SKYWALKER_MOTOR_INVERTED = true;
        public static final double DEADBAND = 0.25;
        public static final double UPSPEED = .50;
        public static final double DOWNSPEED = -.50;
        public static final double SKYWALKER_TENSION_SPEED = 0;
    }

    public static class DriveTrain {
        public static final double DEADBAND = 0.25;
        public static final double SPEED_SENSITIVITY = 0.9;
        public static final double SPEED_SENSITIVITY_LOW = 1;
        public static final double ROTATION_SENSITIVITY = 0.5;
        public static final double CREEP_FACTOR = 0.25;

        public static final int CPR = 8192;
        public static final double ENCODER_CONVERSION = 6.85714286;

        public static final double WIDTH = Units.inchesToMeters(29);

        public static final double KS_VOLTS = 0.172;
        public static final double KV_VOLTSPR = 2.46;
        public static final double KA_VOLTSQPR = 0.355;

        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

        public static final double KP_DRIVE_VELOCITY = 3.25;

        public static final boolean LEFT_MOTORS_INVERTED = false;
        public static final boolean RIGHT_MOTORS_INVERTED = true;

        public static final long LOCK_TIME = 80;
        public static final long DROPOUT_TIME = 100;
        public static final long SEEK_TIME = 500;
        public static final int MAX_SPEED_IPS = 156;
        public static final double MAX_SPEED_MPS = Units.inchesToMeters(MAX_SPEED_IPS);
        public static final double CAP_SPEED_IPS = .8 * MAX_SPEED_IPS;
        public static final double MAX_ACCELERATION_IPSS = CAP_SPEED_IPS / 2;
        public static final double MAX_ACCEL_MPS = Units.inchesToMeters(MAX_ACCELERATION_IPSS);
        public static final double MAX_JERK_IPSSS = CAP_SPEED_IPS;
        public static final double RAMP_RATE = 0.1;
        public static final double RAMP_RATE_LOWGEAR = 1.25; // Seconds from 0 to full in low gear
        public static final double RAMP_INCREMENT_LOWGEAR =  UPDATE_PERIOD / RAMP_RATE_LOWGEAR; // Speed increment allowed per cycle
        public static final int STALL_CURRENT_LIMIT = 50;
        public static final int FREE_CURRENT_LIMIT = 60;
        public static final double SECONDARY_LIMIT = 90;

        public static final double SLOW_ZONE_COMP = 30;
        public static final double MEDIUM_ZONE_COMP = 70;

        public static final double SLOW_SPEED_COMP = 0.4;
        public static final double MEDIUM_SPEED_COMP = 0.6;

        public static final double kP = 0.05;
        public static final double kI = 0.00;
        public static final double kD = 0.0015;
        public static final double ANGLE_TOLERANCE = 1.5;
        public static final double ROTATION_SENSITIVITY_HIGH_GEAR = .8;
        public static final double ROTATION_SENSITIVITY_LOW_GEAR = .5;
        public static final double TURNING_SENSITIVITY_HIGH_GEAR = .5;
        public static final double TURNING_SENSITIVITY_LOW_GEAR = .3;
        public static final double SPEED_LIMIT = 0.9;
        public static final double DISTANCE_TOLERANCE = 2.0;
        public static final double LIMELIGHT_ODOMETRY_ZONE = 150; //inches, we are saying if our distance isnt within this range dont update pose
        public static final double ELEVATOR_LIMIT = 0.3;
    }

    public static class Turret {
        public static final double DEADBAND = 0.1;
        public static final double TOLERANCE = 1.0;
        public static final boolean INVERTED = true;
        public static final boolean SENSOR_PHASE_INVERTED = true;
        public static final double TICKS_TO_DEGREES = 0.08695652173913;
        public static final double MIN_DEGREES = -190;
        public static final double MAX_DEGREES = 131;
        public static final double MAX_VOLTAGE = 12.0;
        public static final int CRUISE_VELOCITY = 5000; // in ticks
        public static final int ACCELERATION = 16000; // in ticks
        public static final double ABS_OFFSET = 239;// if the turret coasts this value changes, need to find a way to set this position.
        public static final double MANUAL_OFFSET = -1.0;


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
        public static final double MIN_DEGREES = 20;
        public static final boolean INVERTED = false;
        public static final boolean SENSOR_PHASE_INVERTED = false;
        public static final double MAX_DEGREES = 70;
        public static final double TICKS_TO_DEGREES = 0.0413080895008606;// 0.038332795242141;
        public static final int CRUISE_VELOCITY = 5000;
        public static final int ACCELERATION = 16000;
        public static final double kP = 2.8;
        public static final double kI = 0.0;
        public static final double kD = 30;
        public static final double kF = 5;
        public static final int I_ZONE =1000;

        public static final double SENSITIVITY = 0.2; //TODO
        public static final double NEAR_TARGET_HOOD_ANGLE_DEGREES = 58;
        public static final double FAR_TARGET_HOOD_ANGLE_DEGREES = Hood.MAX_DEGREES;
        public static final double HEIGHT_TO_DECK = 17;
        public static final double LIMELIGHT_OFFSET_DEGREES = 23;
        public static final double ZEROING_SPEED = -0.85;
        public static final double STOW_DISTANCE = 24;
    }

    public static class OI {
        public static final double AXIS_BUTTON_THRESHHOLD = 0.2;
        public static final long RUMBLE_MILLIS = 250;
        public static final double RUMBLE_INTENSITY = 1.0;
        public static final long RUMBLE_PULSE_TIME = 100;
        public static final int KILL_ALL = 4;
        public static final int OVERRIDE = 8;
        public static final int PANIC = 6;

        public static final int RED_CHANNEL = 2;
        public static final int GREEN_CHANNEL = 3;
        public static final int BLUE_CHANNEL = 1;
    }

    public static class Shifter {
        public static final long STOP_MOTOR_TIME = 60;
        public static final long SHIFT_TIME = 60;

        public static final double SHIFT_UP_THRESHOLD = 50; // in]\[ inches per second graTODO tune
        public static final double SHIFT_DOWN_THRESHOLD = 40; // in inches per second TODO tune

        public static final long AUTO_WAIT_PERIOD = 500;
        public static final long MANUAL_WAIT_PERIOD = 3000;
    }
    public class Limelight {
        public static final double TARGET_HEIGHT = 92;
        public static final double LOW_TARGET_HEIGHT = 17;
        public static final double LIMELIGHT_HEIGHT = 8.125;
        public static final double LIMELIGHT_ANGLE = 0;
        public static final double OVERALL_LATENCY_MILLIS = 11;
    }

    public static class AutoPositions {
        public static double WIDTH_FIELD = Units.inchesToMeters(323.25);
        public static double MID_WIDTH_FIELD = WIDTH_FIELD / 2;
        public static double LENGTH_FIELD = Units.inchesToMeters(629.25);
        public static double MID_LENGTH_FIELD = LENGTH_FIELD / 2;
        public static double TARGET_LINE = MID_LENGTH_FIELD - Units.inchesToMeters(94.66);
        public static double MID_TRENCH = TARGET_LINE + Units.inchesToMeters(66.91);
        public static double AUTO_LINE = Units.inchesToMeters(194.63);
        public static Pose2d TARGET_POSE = new Pose2d(-MID_LENGTH_FIELD, TARGET_LINE, new Rotation2d(0));
        public static Pose2d LOADING_STATION_POSE = new Pose2d(MID_LENGTH_FIELD, TARGET_LINE, new Rotation2d(0));
        public static Pose2d STARTING_POSITION_ONE = new Pose2d(0,0, new Rotation2d(0));
        public static Pose2d BACK_WHEEL_OF_FORTUNE = new Pose2d(0, MID_TRENCH, new Rotation2d(0));
        public static Pose2d FRONT_WHEEL_OF_FORTUNE = new Pose2d(AUTO_LINE, MID_TRENCH, new Rotation2d(0));
        public static Pose2d EIGHT_BALL_STARING = new Pose2d(-AUTO_LINE, TARGET_LINE, new Rotation2d(0));
        public static Pose2d TRENCH_EDGE = new Pose2d(-AUTO_LINE + 1.5, TARGET_LINE + 1.5, new Rotation2d(0));
        public static Pose2d TRENCH_STARTING = new Pose2d(-AUTO_LINE, TARGET_LINE + 1.8288, new Rotation2d(0));

    }

    public class Spinner {
        public static final double MOTOR_PERCENT_SPEED = -1.0;
        public static final double COLOR_TOLERANCE = 0.09;
        public static final double MOTOR_SLOW_PERCENT_SPEED = -0.75;
        public static final int AUTOSPIN_SLOW_AT_WEDGES = 28;
        public static final int AUTOSPIN_STOP_AT_WEDGES = 31;
        public static final double SENSOR_SAMPLE_PERIOD_SECONDS = 0.01;
        public static final boolean ASYNC_COLOR_SAMPLING = true;
        public static final boolean USE_HOMEMADE_COLOR_MATCHING_ALGORITHM = true; // false = use REV Robotic's Algo
        public static final double REV_ALOGORITHM_CONFIDENCE_FACTOR = 0.75; // default is 0.95
    }

    public class RotarySwitch {
        public static final double TOLERANCE = 0.02;
    }

    public class Shooter {
        public static final boolean LEFT_INVERTED = false;
        public static final boolean RIGHT_INVERTED = true;
        public static final double DEADBAND = 0.1;
        public static final double RPM_TOLERANCE = 100; //RPM
        public static final double kP = 0.4;
        public static final double kI = 0.0025;
        public static final double kD = 0.6;
        public static final double kF = 0.05;
        public static final int I_ZONE = 150;
        public static final long TIMEOUT = 15;

        public static final double IDLE_SHOOTER_SPEED_PERCENT = 0.5;  /* TBD RPMs INSTEAD OF PERCENT */
        public static final double NEAR_TARGET_SHOOTER_SPEED_PERCENT = 4000;   /* TBD RPMs INSTEAD OF PERCENT */
        public static final double FAR_TARGET_SHOOTER_SPEED_PERCENT = 5500;   /* TBD RPMs INSTEAD OF PERCENT */
        public static final double TICKS_TO_ROTATIONS = 2048;
        public static final double GEAR_RATIO = 1.25;
        public static final double WHEEL_RADIUS = 3.0;
        public static final double DRAG_CONSTANT = 0.2;
    }

    public class Indexer {
        public static final boolean INVERTED = false;
        public static final double ADVANCE_SPEED = .75; // TODO: Need a real value here!
        public static final double AGITATOR_SPEED = -1.0;
        public static final double SERVO_FORWARD = 1.0;
        public static final double SERVO_STOPPED = 0.51;
        public static final double SERVO_BACKWARDS = 0.00;
    }

    public class Auto {
        public static final long AUTO_SHOOT_DELAY = 1500;
        public static final long AUTO_SHOOT_RUNON = 4000;

        public class Drive {
            public static final double SPEED = 1.0;
            public static final double MIN_SPEED = 0.25;
            public static final double MIN_TRACK_DISTANCE = 2.0;
            public static final int MAX_GARBAGE = 5;
            public static final double STEER_K = 0.015;
            public static final double MAX_IMU_ANGLE = 180.0;
            public static final double MIN_IMU_ANGLE = -MAX_IMU_ANGLE;
        }
    }

    public class AutoDrivePath {
        public static final double K_TURN = 0.1;
    }
    public class DriveStraight {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.001;
        public static final double kP_ANGLE = 0.001;
        public static final double kI_ANGLE = 0.000;
        public static final double kD_ANGLE = 0.01;
        public static final double ANGLE_TOLERANCE = 0.25;
    }

    public class Lights {
        public static final double SOLID_BLUE = 0.87;
        public static final double PULSING_BLUE = -0.09;
        public static final double BEATING_BLUE = 0.23;

        public static final double SOLID_RED = 0.61;
        public static final double PULSING_RED = -0.11;
        public static final double BEATING_RED = 0.25;

        public static final double SOLID_GREEN = 0.77;
        public static final double PULSING_GREEN = 0.77; // replace
        public static final double BEATING_GREEN = 0.00; // unused

        public static final double SOLID_PURPLE = 0.91;
        public static final double PULSING_PURPLE = 0.05;
        public static final double BEATING_PURPLE = 0.00;

        public static final double SOLID_ORANGE = 0.06;
        public static final double PULSING_ORANGE = 0.07;
        public static final double BEATING_ORANGE = 0.08;

        public static final double SOLID_YELLOW = 0.69;
        public static final double PULSING_YELLOW = 0.10;
        public static final double BEATING_YELLOW = 0.11;

        public static final double SOLID_BLACK = 0.99;

        public static final double SOLID_HOT_PINK = 0.57;

        public static final double CONFETTI = -0.87;

        public static final double BLEND_1 = -0.03;
        public static final double SCANNING_1 = -0.01;
        public static final double CHASING_1 = 0.01;
        public static final double SLOW_BEAT_1 = 0.03;
        public static final double MEDIUM_BEAT_1 = 0.05;
        public static final double FAST_BEAT_1 = 0.07;
        public static final double BREATH_SLOW_1 = 0.09;
        public static final double BREATH_FAST_1 = 0.11;
        public static final double SHOT_1 = 0.13;
        public static final double STROBE_1 = 0.15;

        public static final double BLEND_2 = 0.17;
        public static final double SCANNING_2 = 0.19;
        public static final double CHASING_2 = 0.21;
        public static final double SLOW_BEAT_2 = 0.23;
        public static final double MEDIUM_BEAT_2 = 0.25;
        public static final double FAST_BEAT_2 = 0.27;
        public static final double BREATH_SLOW_2 = 0.29;
        public static final double BREATH_FAST_2 = 0.31;
        public static final double SHOT_2 = 0.33;
        public static final double STROBE_2 = 0.35;


        public static final double WHITE_SHOT = -0.81;
        public static final double SOLID_WHITE = 0.93;
    }
}
