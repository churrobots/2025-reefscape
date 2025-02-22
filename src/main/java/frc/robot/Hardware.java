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
    public static final double simCarriageMass = 0.01;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.75); // 22T #25
    public static final double minHeightInMeters = 0.0;
    public static final double maxHeightInMeters = 0.35;
  }

  public final class Elbow {
    // FIXME: we should make this a lower number to make CAN more efficient? (i
    // think there was some notes on chief delphi about this?)
    public static final int neoMotorCAN = 31;
    public static final double gearboxReuction = 75; // check this
  }

  public final class Drivetrain {
    // TODO: consider upping the max speed again
    public static final double maxSpeedMetersPerSecond = 4; // theoretical max is 6 but we saw 4 in testing
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