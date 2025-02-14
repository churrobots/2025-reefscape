// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int neoMotorCAN = 17;
  }

  public final class Drivetrain {
    public static final double maxSpeedMetersPerSecond = 6.04;
    public static final String swerveConfigDeployPath = switch (robotName) {
      case ROBOT_CANELO -> "yagsl-configs/canelo";
      case ROBOT_ALPHA -> "yagsl-configs/alpha";
      case ROBOT_BETA -> "yagsl-configs/beta";
      case ROBOT_SIMULATION -> "yagsl-configs/alpha";
      default -> "yagsl-configs/beta";
    };
    public static final boolean debugTelemetry = switch (robotName) {
      case ROBOT_BETA -> false;
      case ROBOT_ALPHA -> false;
      default -> true;
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
    public static final boolean useLowQualityCamera = switch (robotName) {
      case ROBOT_ALPHA -> true;
      default -> false;
    };
  }

}