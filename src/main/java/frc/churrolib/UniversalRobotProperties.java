// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import edu.wpi.first.math.system.plant.DCMotor;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class UniversalRobotProperties {
  // TODO: ensure we have a way to capture the physical properties from YAGSL
  // https://docs.yagsl.com/configuring-yagsl/configuration/physical-properties-configuration
  public UniversalRobotProperties(String pathToYAGSLConfig, double maxSpeedMetersPerSecond) {

  }

  public RobotConfig getAsPathPlannerConfig() {
    // FIXME: implement this
    double wheelRadius = 0;
    double maxDriveVelocity = 0;
    double wheelCOF = 0;
    DCMotor driveMotor = DCMotor.getNEO(1);
    double driveCurrentLimit = 0;
    double robotMassKg = 0;
    double robotMOI = 0;
    double trackwidthMeters = 0;
    int numMotors = 0;
    ModuleConfig moduleConfig = new ModuleConfig(
        wheelRadius,
        maxDriveVelocity,
        wheelCOF,
        driveMotor,
        driveCurrentLimit,
        numMotors);
    return new RobotConfig(robotMassKg, robotMOI, moduleConfig, trackwidthMeters);
  }

  public SwerveDrive getAsYAGSLDrive() {
    // FIXME: implement this
    return null;
  }

}
