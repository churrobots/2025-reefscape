// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

public final class Hardware {

  // This helps us differentiate multiple robots. Each robot stores
  // a persisted integer in NetworkTables, so that we know which
  // robot we got deployed to, in case there are specific constants
  // that have different values between those robots.
  public static final long ROBOT_CANELO = 1;
  public static final long ROBOT_ALPHA = 2;
  public static final long ROBOT_COMPETITION = 3;
  public static final long robot = NetworkTableInstance
      .getDefault()
      .getTable("churrobots")
      .getEntry("robot_id")
      .getInteger(ROBOT_CANELO);

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

  public final class TemplateSwerve {
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