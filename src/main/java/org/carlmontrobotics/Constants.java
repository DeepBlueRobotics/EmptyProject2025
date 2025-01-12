package org.carlmontrobotics;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }
    public static final class OI {
        public static final class Driver {
            public static final int port = 0;
        }
        public static final class Manipulator {
            public static final int port = 1;
        }

        public static final class ElevatorC {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double pidTolerance = 0;

            public static final double MAX_VEL = 0;
            public static final double MAX_ACCEL = 0;

            public static final double posBase = 0;
            public static final double posL1 = 1;
            public static final double posL2 = 2;
            public static final double posL3 = 3;
            public static final double posL4 = 4;

            public static final double gearReduction = 0.2;
            public static final double rotationIntoHeight = 2; //inches
        }
    }
}
