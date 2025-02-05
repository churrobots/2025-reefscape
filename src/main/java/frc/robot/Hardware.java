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
  public static final String ROBOT_COMPETITION = "competition";
  public static final String robotName = NetworkTableInstance
      .getDefault()
      .getEntry("robotName")
      .getString(RobotBase.isSimulation() ? ROBOT_SIMULATION : ROBOT_COMPETITION);

  public final class Shooter {
    public static final int falconMotorCAN = 14;
    public static final double gearboxReduction = 5.0;
    public static final double simMomentOfInertia = 0.01;
    public static boolean isEnabled = switch (robotName) {
      case ROBOT_SIMULATION -> true;
      default -> false;
    };
  }

  public final class Elevator {
    public static final int leaderFalconMotorCAN = 15;
    public static final int followerFalconMotorCAN = 16;
    public static final double gearboxReduction = 5.0;
    public static final double simCarriageMass = 0.01;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.273); // 16T #25
    public static final double minHeightInMeters = 0.0;
    public static final double maxHeightInMeters = 0.5;
    public static boolean isEnabled = switch (robotName) {
      case ROBOT_SIMULATION -> true;
      default -> false;
    };
  }

  public final class Elbow {
    public static final int neoMotorCAN = 17;
    // TODO: Get actual CAN id
    public static boolean isEnabled = switch (robotName) {
      case ROBOT_SIMULATION -> true;
      default -> false;
    };
  }

  public final class Drivetrain {
    // NOTE: eventually we will migrate over to the YAGSL drivetrain, but for now
    // we are keeping both so we can switch back in the worst case scenario
    public static final boolean useYAGSL = switch (robotName) {
      case ROBOT_CANELO -> false;
      default -> true;
    };
    public static final double maxSpeedMetersPerSecond = 6.04;
  }

  public final class DrivetrainWithYAGSL {
    public static final String swerveConfigDeployPath = switch (robotName) {
      case ROBOT_CANELO -> "yagsl-configs/canelo";
      case ROBOT_ALPHA -> "yagsl-configs/alpha";
      case ROBOT_COMPETITION -> "yagsl-configs/competition";
      case ROBOT_SIMULATION -> "yagsl-configs/alpha";
      default -> "yagsl-configs/competition";
    };
    public static final boolean debugTelemetry = switch (robotName) {
      case ROBOT_COMPETITION -> false;
      default -> true;
    };
  }

  // NOTE: once we adopt YAGSL we won't need these template vars
  // to config the subsystem, since all the configs are in YAGSL
  public final class DrivetrainWithTemplate {

    public static final int frontLeftTurningMotorCAN = 1;
    public static final int frontLeftDrivingMotorCAN = 5;

    public static final int frontRightTurningMotorCAN = 2;
    public static final int frontRightDrivingMotorCAN = 6;

    public static final int rearLeftTurningMotorCAN = 3;
    public static final int rearLeftDrivingMotorCAN = 7;

    public static final int rearRightTurningMotorCAN = 4;
    public static final int rearRightDrivingMotorCAN = 8;

    public static final int pigeonGyroCAN = 9;

    // Tuning values
    public static final double kWheelDiameterMeters = 0.0831;
    public static final double kDrivingP = .08;// .04
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kTurningP = 2;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    // Note that the module itself has 45 teeth on the wheel's bevel gear, 22 teeth
    // on the first-stage spur gear, 15 teeth on the bevel pinion.
    public static final int kDrivingMotorPinionTeeth = 16;
    public static final int kWheelBevelTeeth = 45;
    public static final int kFirstStageSpurTeeth = 20;
    public static final int kBevelPinionTeeth = 15;

    // Neo motors are 5676 max RPM
    public static final double kMotorFreeSpeedRpm = 5676;

    // Other items we need centrally to manage simulation.
    public static final double wheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double drivingMotorGearboxReduction = ((double) kWheelBevelTeeth
        * kFirstStageSpurTeeth)
        / ((double) kDrivingMotorPinionTeeth
            * kBevelPinionTeeth);
    public static final double drivingEncoderVelocityFactorInMetersPerSecond = (wheelCircumferenceMeters
        / drivingMotorGearboxReduction) / 60.0; // meters per second because native units are RPM

    // Turning motor reduction from "Azimuth Ratio" of MAXSwerve module spec:
    // https://www.revrobotics.com/rev-21-3005/
    public static final double turningMotorGearboxReduction = 9424 / 203;
    public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    // because native units are RPM

    // Chassis configuration
    public static final double kTrackWidth = switch (robotName) {
      case ROBOT_CANELO -> Units.inchesToMeters(18.5);
      default -> Units.inchesToMeters(26.5);
    };
    public static final double kWheelBase = switch (robotName) {
      case ROBOT_CANELO -> Units.inchesToMeters(23.5);
      default -> Units.inchesToMeters(26.5);
    };

    // Offsets of the turning motor of each module, relative to chassis (radians).
    public static final double kFrontLeftChassisAngularOffset = switch (robotName) {
      case ROBOT_CANELO -> -Math.PI / 2;
      default -> 0;
    };
    public static final double kFrontRightChassisAngularOffset = switch (robotName) {
      case ROBOT_CANELO -> 0;
      default -> 0;
    };
    public static final double kRearLeftChassisAngularOffset = switch (robotName) {
      case ROBOT_CANELO -> Math.PI;
      default -> Math.PI;
    };
    public static final double kRearRightChassisAngularOffset = switch (robotName) {
      case ROBOT_CANELO -> Math.PI / 2;
      default -> Math.PI;
    };

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kDirectionSlewRate = 3.6; // radians per second
    public static final double kMagnitudeSlewRate = 4.5; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 6; // percent per second (1 = 100%)

    // Current limits
    public static final int kDrivingMotorCurrentLimitInAmps = 50;
    public static final int kTurningMotorCurrentLimitInAmps = 20;
  }

  public static class DriverStation {
    public static final int driverXboxPort = 0;
    public static final int operatorXboxPort = 1;
    public static final double driverXboxDeadband = 0.1;
    public static final double fastDriveScale = 1.0;
    public static final double slowDriveScale = 0.25;
  }

}