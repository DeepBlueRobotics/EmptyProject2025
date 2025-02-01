package org.carlmontrobotics;

import org.carlmontrobotics.lib199.swerve.SwerveConfig;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
    public static final double g = 9.81;

    public static final class OI {
        public static final class Driver {
			public static final int port = 0;

			public static final int slowDriveButton = Button.kLeftBumper.value;
			public static final int resetFieldOrientationButton = Button.kRightBumper.value;
			public static final int toggleFieldOrientedButton = Button.kStart.value;

			public static final int rotateFieldRelative0Deg = Button.kY.value;
			public static final int rotateFieldRelative90Deg = Button.kB.value;
			public static final int rotateFieldRelative180Deg = Button.kA.value;
			public static final int rotateFieldRelative270Deg = Button.kX.value;
		}
        public static final double JOY_THRESH = 0.2;
    }

    public static final class Drivetrainc{
        public static final double wheelBase = Units.inchesToMeters(19.75);
        
    public static final double trackWidth = Units.inchesToMeters(23.75);
    // "swerveRadius" is the distance from the center of the robot to one of the
    // modules
    public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
    // The gearing reduction from the drive motor controller to the wheels
    // Gearing for the Swerve Modules is 6.75 : 1
    public static final double driveGearing = 6.75;
    // Turn motor shaft to "module shaft"
    public static final double turnGearing = 150 / 7;

    public static final double driveModifier = 1;
    public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36 / 7.65 /*
                                                                                                * empirical
                                                                                                * correction
                                                                                                */;
    public static final double mu = 1; /* 70/83.2; */

    public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60; // radians/s
    // Angular speed to translational speed --> v = omega * r / gearing
    public static final double maxSpeed = NEOFreeSpeed * (wheelDiameterMeters / 2.0) / driveGearing;
    public static final double maxForward = maxSpeed; // todo: use smart dashboard to figure this out
    public static final double maxStrafe = maxSpeed; // todo: use smart dashboard to figure this out
    // seconds it takes to go from 0 to 12 volts(aka MAX)
    public static final double secsPer12Volts = 0.1;

    // maxRCW is the angular velocity of the robot.
    // Calculated by looking at one of the motors and treating it as a point mass
    // moving around in a circle.
    // Tangential speed of this point mass is maxSpeed and the radius of the circle
    // is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
    // Angular velocity = Tangential speed / radius
    public static final double maxRCW = maxSpeed / swerveRadius;

    public static final boolean[] reversed = { false, false, false, false };
    // public static final boolean[] reversed = {true, true, true, true};
    // Determine correct turnZero constants (FL, FR, BL, BR)
    public static final double[] turnZeroDeg = RobotBase.isSimulation() ? new double[] {-90.0, -90.0, -90.0, -90.0 }
            : new double[] { -48.6914, 63.3691, 94.1309, -6.7676 };/* real values here */

    // kP, kI, and kD constants for turn motor controllers in the order of
    // front-left, front-right, back-left, back-right.
    // Determine correct turn PID constants
    public static final double[] turnkP = { 51.078, 60.885, 60.946, 60.986 }; // {0.00374, 0.00374, 0.00374,
                                                                                // 0.00374};
    public static final double[] turnkI = { 0, 0, 0, 0 };
    public static final double[] turnkD = { 0/* dont edit */, 0.5, 0.42, 1 }; // todo: use d
    // public static final double[] turnkS = {0.2, 0.2, 0.2, 0.2};
    public static final double[] turnkS = { 0.13027, 0.17026, 0.2, 0.23262 };

    // V = kS + kV * v + kA * a
    // 12 = 0.2 + 0.00463 * v
    // v = (12 - 0.2) / 0.00463 = 2548.596 degrees/s
    public static final double[] turnkV = { 2.6532, 2.7597, 2.7445, 2.7698 };
    public static final double[] turnkA = { 0.17924, 0.17924, 0.17924, 0.17924 };

    // kP is an average of the forward and backward kP values
    // Forward: 1.72, 1.71, 1.92, 1.94
    // Backward: 1.92, 1.92, 2.11, 1.89
    // Order of modules: (FL, FR, BL, BR)
    public static final double[] drivekP = new double[] { 1.75, 1.75, 1.75, .75 }; // {1.82/100, 1.815/100, 2.015/100,
                                                                        // 1.915/100};
    public static final double[] drivekI = { 0, 0, 0, 0 };
    public static final double[] drivekD = { 0, 0, 0, 0 };
    public static final boolean[] driveInversion = (new boolean[] { true, false, true, false });
    public static final boolean[] turnInversion = { true, true, true, true };
    // kS
    public static final double[] kForwardVolts = { 0.26744, 0.31897, 0.27967, 0.2461 };
    public static final double[] kBackwardVolts = kForwardVolts;

    public static final double[] kForwardVels = { 2.81, 2.9098, 2.8378, 2.7391 };
    public static final double[] kBackwardVels = kForwardVels;
    public static final double[] kForwardAccels = { 1.1047 / 2, 0.79422 / 2, 0.77114 / 2, 1.1003 / 2 };
    public static final double[] kBackwardAccels = kForwardAccels;

    public static final double autoMaxSpeedMps = 0.35 * 4.4; // Meters / second
    public static final double autoMaxAccelMps2 = mu * g; // Meters / seconds^2
    public static final double autoMaxVolt = 10.0; // For Drivetrain voltage constraint in RobotPath.java
    // The maximum acceleration the robot can achieve is equal to the coefficient of
    // static friction times the gravitational acceleration
    // a = mu * 9.8 m/s^2
    public static final double autoCentripetalAccel = mu * g * 2;

    public static final boolean isGyroReversed = true;

    // PID values are listed in the order kP, kI, and kD
    public static final double[] xPIDController = new double[] { 2, 0.0, 0.0 };
    public static final double[] yPIDController = xPIDController;
    public static final double[] thetaPIDController = new double[] {0.05, 0.0, 0.00};

    public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, mu,
            autoCentripetalAccel, kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels,
            kBackwardAccels, drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZeroDeg,
            driveInversion, reversed, driveModifier, turnInversion);

    // public static final Limelight.Transform limelightTransformForPoseEstimation =
    // Transform.BOTPOSE_WPIBLUE;

    // #endregion

    // #region Ports

    public static final int driveFrontLeftPort = 11; 
    public static final int driveFrontRightPort = 13;
    public static final int driveBackLeftPort = 14; 
    public static final int driveBackRightPort = 17; 

    public static final int turnFrontLeftPort = 12; 
    public static final int turnFrontRightPort = 20; 
    public static final int turnBackLeftPort = 15; 
    public static final int turnBackRightPort = 16; 

    public static final int canCoderPortFL = 0;
    public static final int canCoderPortFR = 3;
    public static final int canCoderPortBL = 2;
    public static final int canCoderPortBR = 1;

    // #endregion

    // #region Command Constants

    public static double kNormalDriveSpeed = 1; // Percent Multiplier
    public static double kNormalDriveRotation = 0.5; // Percent Multiplier
    public static double kSlowDriveSpeed = 0.4; // Percent Multiplier
    public static double kSlowDriveRotation = 0.250; // Percent Multiplier

    //baby speed values, i just guessed the percent multiplier. TODO: find actual ones we wana use
    public static double kBabyDriveSpeed = 0.3;
    public static double kBabyDriveRotation = 0.2;
    public static double kAlignMultiplier = 1D / 3D;
    public static final double kAlignForward = 0.6;

    public static final double wheelTurnDriveSpeed = 0.0001; // Meters / Second ; A non-zero speed just used to
                                                                // orient the wheels to the correct angle. This
                                                                // should be very small to avoid actually moving the
                                                                // robot.

    public static final double[] positionTolerance = { Units.inchesToMeters(.5), Units.inchesToMeters(.5), 5 }; // Meters,
                                                                                                                // Meters,
                                                                                                                // Degrees
    public static final double[] velocityTolerance = { Units.inchesToMeters(1), Units.inchesToMeters(1), 5 }; // Meters,
                                                                                                                // Meters,
                                                                                                                // Degrees/Second

    // #endregion
    // #region Subsystem Constants

    // "swerveRadius" is the distance from the center of the robot to one of the
    // modules
    public static final double turnkP_avg = (turnkP[0] + turnkP[1] + turnkP[2] + turnkP[3]) / 4;
    public static final double turnIzone = .1;

    public static final double driveIzone = .1;

    public static final class Autoc {
        public static final ReplanningConfig replanningConfig = new ReplanningConfig( /*
                                                                                        * put in
                                                                                        * Constants.Drivetrain.Auto
                                                                                        */
                false, // replan at start of path if robot not at start of path?
                false, // replan if total error surpasses total error/spike threshold?
                1.5, // total error threshold in meters that will cause the path to be replanned
                .8 // error spike threshold, in meters, that will cause the path to be replanned
        );
        public static final PathConstraints pathConstraints = new PathConstraints(1.54, 6.86, 2 * Math.PI,
                2 * Math.PI); // The constraints for this path. If using a differential drivetrain, the
                                // angular constraints have no effect.
        }
    }
}
