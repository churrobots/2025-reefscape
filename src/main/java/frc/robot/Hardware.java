// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

public final class Hardware {

  // This helps us differentiate multiple robots. Each robot stores
  // a persisted string in NetworkTables, so that we know which
  // robot we got deployed to, in case there are specific constants
  // that have different values between those robots.
  public static final String robotName = NetworkTableInstance
      .getDefault()
      .getEntry("robotName")
      .getString(null);
  public static final String ROBOT_CANELO = "canelo";
  public static final String ROBOT_ALPHA = "alpha";
  public static final String ROBOT_COMPETITION = "competition";

  public final class Intake {
    public static final int brushedMotorCAN = 11;
    public static final double gearboxReduction = 1.0;
  }

  public final class Shooter {
    public static final int falconMotorCAN = 10;
    public static final double gearboxReduction = 5.0;
    public static final double simMomentOfInertia = 0.01;
  }

  public final class Elevator {
    public static final int leaderFalconMotorCAN = 18;
    public static final int followerFalconMotorCAN = 24;
    public static final double gearboxReduction = 5.0;
    public static final double simCarriageMass = 0.01;
    public static final double minHeight = 0.0;
    public static final double maxHeight = 1.0;
  }

  // NOTE: once we adopt YAGSL we won't need these template vars
  // to config the subsystem, since all the configs are in YAGSL
  public final class RevMAXSwerveTemplate {
    public static final boolean isUsable = switch (robotName) {
      case ROBOT_CANELO -> true;
      default -> false;
    };
    public static final int frontLeftTurningMotorCAN = 1;
    public static final int rearLeftTurningMotorCAN = 3;
    public static final int frontRightTurningMotorCAN = 2;
    public static final int rearRightTurningMotorCAN = 4;
    public static final int frontLeftDrivingMotorCAN = 5;
    public static final int rearLeftDrivingMotorCAN = 7;
    public static final int frontRightDrivingMotorCAN = 6;
    public static final int rearRightDrivingMotorCAN = 8;
    public static final int pigeonGyroCAN = 9;
  }

}