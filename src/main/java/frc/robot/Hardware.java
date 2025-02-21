// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

public final class Hardware {

  // This helps us differentiate multiple robots. Each robot stores
  // a persisted string in NetworkTables, so that we know which
  // a persisted string in NetworkTables, so that we know which
  // robot we got deployed to, in case there are specific constants
  // that have different values between those robots.
  public static final String ROBOT_SIMULATION = "simulation";
  public static final String ROBOT_CANELO = "canelo";
  public static final String ROBOT_ALPHA = "alpha";
  public static final String ROBOT_BETA = "beta";
  public static final String robotName = NetworkTableInstance
      .getDefault()
      .getEntry("robotName")
      .getString(RobotBase.isSimulation() ? ROBOT_SIMULATION : ROBOT_BETA);

  public final class Pipeshooter {
    public static final int falconMotorCAN = 14;
    public static final double gearboxReduction = 5.0;
    public static final double simMomentOfInertia = 0.01;
  }

  public final class Elevator {
    public static final int leaderFalconMotorCAN = 15;
    public static final int followerFalconMotorCAN = 16;
    public static final double gearboxReduction = 4.67; // CIM PLE Gearbox
                                                        // https://www.andymark.com/products/cimple-box-single-stage-gearbox
    public static final double kCurrentLimit = 80;
    public static final double simCarriageMass = 0.01;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.75); // 22T #25

    // PID numbers from:
    // https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/basic-pid-control.html
    public static final double kS = 0.0; // static friction feed-forward component.
    public static final double kV = 2.83; // motor-specific, represents the motor's velocity per volt. The reciprocal is
    // used as the feed-forward parameter in velocity-based control loops.
    public static final double kA = 0.03; //
    public static final double kP = 2; // proportional
    public static final double kI = 0.0; // integral
    public static final double kD = 0.0; // derivative
    public static final double kG = 0.31; // gravity

    // Maximum velocity in rotations per second of the post-gearbox sprocket (NOT of
    // the motor).
    public static final double kMaxVelocity = 100 * gearboxReduction;
    public static final double kMaxAcceleration = 50 * gearboxReduction;
    public static final double kMaxJerk = 300 * gearboxReduction;

    // Desired Vertical Travel in meters for each position. Heights are specified
    // relative to the base position. Total elevator travel ranges from 0 to ~35cm.
    public static final double minHeightInMeters = 0.0;
    public static final double maxHeightInMeters = 0.33;
    public static final double kBaseHeight = 0.0; // initial height, also the height where we receive coral.
    public static final double kL1Height = 0.11; // height to score in L1 trough
    public static final double kL2Height = 0.22; // height to score in L2
    public static final double kL3Height = 0.33; // height to score in L3
  }

  public final class Elbow {
    // TODO: we should make this a lower number to make CAN more efficient? (i
    // think there was some notes on chief delphi about this?)
    public static final int neoMotorCAN = 31;
    public static final double gearboxReduction = 75;
    public static final boolean encoderIsInverted = true;
    public static final boolean motorIsInverted = true;
    public static final double minOutput = -1.0;
    public static final double maxOutput = 1.0;
    public static final int currentLimitInAmps = 30;
    public static final double kP = 20;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double minRotations = 0;
    public static final double maxRotations = 0.6;
    public static final double targetToleranceInRotations = 0.02;
    public static final double speedForResettingPosition = 0.05;

    // These are the angles appropriate for different positions.
    public static final double receivingRotations = 0.05;
    public static final double aimAtReefRotations = 0.50;
    public static final double aimAtAlgaeRotations = 0.30;

    // These govern the ranges of motion for being tucked and extended, to prevent
    // crashing with reef and crossbar.
    public static final double maxHeightToKeepTucked = 0.08;
    public static final double maxTuckedRotations = 0.11;
    public static final double minExtendedRotations = 0.15;
  }

  public final class Drivetrain {
    public static final double maxSpeedMetersPerSecond = 6.04;
    // TODO: Update the MOIs to match the robots.
    public static final double robotMOI = switch (robotName) {
      case ROBOT_CANELO -> 6.0;
      case ROBOT_ALPHA -> 2.0;
      case ROBOT_BETA -> 6.0;
      case ROBOT_SIMULATION -> 2.0;
      default -> 6.0;
    };
    public static final String swerveConfigDeployPath = switch (robotName) {
      case ROBOT_CANELO -> "yagsl-configs/canelo";
      case ROBOT_ALPHA -> "yagsl-configs/alpha";
      case ROBOT_BETA -> "yagsl-configs/beta";
      case ROBOT_SIMULATION -> "yagsl-configs/beta";
      default -> "yagsl-configs/beta";
    };
    // PathPlanner config values
    // double wheelRadius = 0;
    // double maxDriveVelocity = 0;
    // double wheelCOF = 0;
    // DCMotor driveMotor = DCMotor.getNEO(1);
    // double driveCurrentLimit = 0;
    // double robotMassKg = 0;
    // double robotMOI = 0;
    // double trackwidthMeters = 0;
    // int numMotors = 0;
  }

  public final class Vision {
    public static final boolean isEnabled = switch (robotName) {
      case ROBOT_CANELO -> false;
      case ROBOT_ALPHA -> false;
      case ROBOT_BETA -> false;
      case ROBOT_SIMULATION -> true;
      default -> false;
    };

    // TODO: Update the Transform3d to match the camera position on the bot
    // Currently it is set to a camera mounted facing forward, 0.5 meters forwards
    // of center, 0.0 meters right of center, 0.5 meters up from center
    public static final Transform3d robotToCam1 = switch (robotName) {
      case ROBOT_ALPHA -> new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
      case ROBOT_BETA -> new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
      default -> new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    };
  }

  public final class LEDLights {
    public static final int ledPWM = 9;
    public static final int leftLEDCount = 144;
    public static final int rightLEDCount = 144;
  }

  public static class DriverStation {
    public static final int driverXboxPort = 0;
    public static final int operatorXboxPort = 1;
    public static final int driverSimulationXboxPort = 2;
    public static final double driverXboxDeadband = 0.1;
    public static final double fastDriveScale = 1.0;
    public static final double slowDriveScale = 0.15;
    public static final boolean mechanismsAreInTestMode = switch (robotName) {
      case ROBOT_BETA -> true;
      default -> false;
    };
  }

  public static class Diagnostics {
    public static final boolean debugMemoryLeaks = switch (robotName) {
      case ROBOT_ALPHA -> true;
      default -> false;
    };
    public static final boolean debugTelemetry = switch (robotName) {
      case ROBOT_BETA -> false;
      case ROBOT_ALPHA -> false;
      default -> true;
    };
  }

}