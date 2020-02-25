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
            public static final int LEFT_MASTER = 3;
            public static final int RIGHT_MASTER = 20;
            public static final int LEFT_FOLLOWER = 2;
            public static final int RIGHT_FOLLOWER = 1;
            public static final int INTAKE_NEO = 9;
            public static final int ELEVATOR_NEO = 8;
            public static final int WINCH_NEO = 12;
            public static final int INDEXER = 11;
        }

        public static class TALONFX {
            public static final int RIGHT_SHOOTER = 14;
            public static final int LEFT_SHOOTER = 15;
        }
        public static class TALONSRX {
            public static final int TURRET = 13;
            public static final int HOOD = 6;
        }
        public static class VICTORSPX {
            public static final int SPINNER_SKYWALKER = 7;
            public static final int AGITATOR = 10;
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
            public static final int SHORT_LED_STRIP = 0;
            public static final int LONG_LED_STRIP = 1;
            public static final int AGITATOR = 2;
            public static final int AGITATOR1 = 3;
            public static final int AGITATOR2 = 4;
            public static final int AGITATOR3 = 5;
            public static final int AGITATOR4 = 6;
            public static final int AGITATOR5 = 7;
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
            public static final int SPINNER_DEPLOY = 5;
            public static final int SPINNER_STOW = 4;
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
        }

        /**
         * There should be an entry here for each DIO port, preferrably in numerical order.
         * Note that for DIO only one device can connect to each port, so the numbers should be unique.
         */
        public static class DIO {
            public static final int TOP_IR = 0;
            public static final int MID_IR = 1;
            public static final int BOTTOM_IR = 2;
            public static final int HOOD_HALL = 6;
            public static final int UP_IR = 3;
            public static final int DOWN_IR = 4;

            public static final int FRONT_SHARK = 8;
            public static final int REAR_SHARK = 9;
        /* Example:
        public static final int ARM_FRONT_LIMIT = 0;
        */
        }


 }

