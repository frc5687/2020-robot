package org.frc5687.infiniterecharge.robot;

public class RobotMap {

    /**
     * There should be an entry here for each CAN device, preferrably grouped by device type and then in numerical order.
     * Note that for CAN, ids must be unique per device type, but not across types.
     * Thus, you cannot have two SparkMax controllers with Id 0, but you can have a SparkMax with Id 0 and a TalonSRX with Id 0.
     */
    public static class CAN {

        public static class SPARKMAX {
            /*  Example:
                public static final int LEFT_MASTER_SPARK= 1;
                */
            public static final int LEFT_MASTER = 4;
            public static final int RIGHT_MASTER = 1;
            public static final int LEFT_SLAVE = 3;
            public static final int RIGHT_SLAVE = 2;
            public static final int INTAKE_NEO = 5;
            public static final int ELEVATOR_NEO = 8;
            public static final int WINCH_NEO = 7;
            public static final int INDEXER = 5;
        }

        public static class TALONFX {
            public static final int RIGHT_SHOOTER = 0;
            public static final int LEFT_SHOOTER = 1;
        }
        public static class TALONSRX {
            public static final int TURRET = 8;
            public static final int HOOD = 7;
        }
        public static class VICTORSPX {
            public static final int SPINNER = 1;
        }

    }

        /**
         * There should be an entry here for each PWM port, preferrably in numerical order.
         * Note that for PWM only one device can connect to each port, so the numbers should be unique.
         */
        public static class PWM {
            /*  Example:
            public static final int ARM_VICTORSP = 0;
            */
            public static final int LEFT_FRONT_DRIVE_MOTOR = 0;
            public static final int LEFT_BACK_DRIVE_MOTOR = 1;
            public static final int RIGHT_FRONT_DRIVE_MOTOR = 2;
            public static final int RIGHT_BACK_DRIVE_MOTOR = 3;
        }

        /**
         * There should be an entry here for each PCM port, preferrably in numerical order.
         * Note that for PCM only one device can connect to each port, so the numbers should be unique.
         */
        public static class PCM {
        /* Example:
        public static final int LEFT_PINCER_OPEN = 5;
        */

            //PCM ports are not in the robot yet, using 0 and 1.
            public static final int SHIFTER_HIGH = 2;
            public static final int SHIFTER_LOW = 3;
            public static final int INTAKE_HIGH = 0;
            public static final int INTAKE_LOW = 1;
            public static final int SPINNER_DEPLOY = 6;
            public static final int SPINNER_STOW = 7;
        }



        /**
         * There should be an entry here for each PDP breaker, preferrably in numerical order.
         * Note that only on device can be connected to each breaker, so the numbers should be unique.
         */
        public static class PDP {
        /* Example:
        public static final int ARM_VICTORSP = 0;
        */
        }

        /**
         * There should be an entry here for each Analgo port, preferrably in numerical order.
         * Note that for Analog only one device can connect to each port, so the numbers should be unique.
         */
        public static class Analog {
            public static final int MODE_SWITCH = 0;
            public static final int SUBSYSTEM_SELECTOR = 1;
        /*
        public static final int ARM_POTENTIOMETER = 7;
         */
        }

        /**
         * There should be an entry here for each DIO port, preferrably in numerical order.
         * Note that for DIO only one device can connect to each port, so the numbers should be unique.
         */
        public static class DIO {
            public static final int TOP_IR = 0;
            public static final int MID_IR = 1;
            public static final int BOTTOM_IR = 2;
            public static final int HOOD_ENCODER = 3;
            public static final int DRIVE_LEFT_B = 4;
            public static final int DRIVE_LEFT_A = 5;
            public static final int DRIVE_RIGHT_B = 6;
            public static final int DRIVE_RIGHT_A = 7;

        /* Example:
        public static final int ARM_FRONT_LIMIT = 0;
        */
        }


 }

