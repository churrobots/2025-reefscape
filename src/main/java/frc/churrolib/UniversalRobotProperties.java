// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import swervelib.SwerveDrive;
import swervelib.parser.json.ModuleJson;
import swervelib.parser.json.PhysicalPropertiesJson;
import swervelib.parser.json.SwerveDriveJson;

public class UniversalRobotProperties {
  private static final double LBS_PER_KG = 2.20462;
  private static final double INCHES_PER_METER = 39.3701;

  final File m_yagslDirectory;
  final double m_maxDriveVelocity;
  final double m_robotMOI;

  // TODO: ensure we have a way to capture the physical properties from YAGSL
  // https://docs.yagsl.com/configuring-yagsl/configuration/physical-properties-configuration
  public UniversalRobotProperties(File yagslDirectory, double maxSpeedMetersPerSecond, double robotMOI) {
    m_yagslDirectory = yagslDirectory;
    m_maxDriveVelocity = maxSpeedMetersPerSecond;
    m_robotMOI = robotMOI;
  }

  public RobotConfig getAsPathPlannerConfig() throws IOException {
    // Read relevant YAGSL config files (see swervelib.parser.SwerveParser)
    checkDirectory(m_yagslDirectory);
    SwerveDriveJson swerveDriveJson = new ObjectMapper()
        .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
        .readValue(new File(m_yagslDirectory, "swervedrive.json"), SwerveDriveJson.class);
    PhysicalPropertiesJson physicalPropertiesJson = new ObjectMapper()
        .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
        .readValue(
            new File(m_yagslDirectory, "modules/physicalproperties.json"),
            PhysicalPropertiesJson.class);
    ModuleJson[] moduleJsons = new ModuleJson[swerveDriveJson.modules.length];
    // Modules are in clockwise order starting from front left
    for (int i = 0; i < moduleJsons.length; i++) {
      File moduleFile = new File(m_yagslDirectory, "modules/" + swerveDriveJson.modules[i]);
      assert moduleFile.exists();
      moduleJsons[i] = new ObjectMapper()
          .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
          .readValue(moduleFile, ModuleJson.class);
    }

    // Use YAGSL config files to setup RobotConfig (see PathPlanner
    // RobotConfig.fromGUISettings)
    double wheelRadiusMeters = inchesToMeters(physicalPropertiesJson.conversionFactors.drive.diameter / 2);
    double wheelCOF = physicalPropertiesJson.wheelGripCoefficientOfFriction;
    double driveCurrentLimit = physicalPropertiesJson.currentLimit.drive;
    double robotMassKg = lbsToKg(physicalPropertiesJson.robotMass);
    int numMotors = 1;

    double gearing = physicalPropertiesJson.conversionFactors.drive.gearRatio;
    DCMotor gearbox = motorFromModule(moduleJsons[0]);
    gearbox = gearbox.withReduction(gearing);

    ModuleConfig moduleConfig = new ModuleConfig(
        wheelRadiusMeters, m_maxDriveVelocity, wheelCOF, gearbox, driveCurrentLimit, numMotors);

    Translation2d[] moduleOffsets = new Translation2d[] {
        new Translation2d(inchesToMeters(moduleJsons[0].location.front), inchesToMeters(moduleJsons[0].location.left)),
        new Translation2d(inchesToMeters(moduleJsons[1].location.front), inchesToMeters(moduleJsons[1].location.left)),
        new Translation2d(inchesToMeters(moduleJsons[2].location.front), inchesToMeters(moduleJsons[2].location.left)),
        new Translation2d(inchesToMeters(moduleJsons[3].location.front), inchesToMeters(moduleJsons[3].location.left)),
    };
    return new RobotConfig(robotMassKg, m_robotMOI, moduleConfig, moduleOffsets);
  }

  private double inchesToMeters(double inches) {
    return inches / INCHES_PER_METER;
  }

  private double lbsToKg(double lbs) {
    return lbs / LBS_PER_KG;
  }

  // Verifies proper YAGSL files exist (swervelib.parser.SwerveParser)
  private void checkDirectory(File directory) {
    assert new File(directory, "swervedrive.json").exists();
    assert new File(directory, "controllerproperties.json").exists();
    assert new File(directory, "modules").exists() && new File(directory, "modules").isDirectory();
    assert new File(directory, "modules/pidfproperties.json").exists();
    assert new File(directory, "modules/physicalproperties.json").exists();
  }

  private DCMotor motorFromModule(ModuleJson moduleJson) {
    String moduleType = moduleJson.drive.type;
    if (moduleType.equals("sparkmax") || moduleType.equals("neo")) {
      return DCMotor.getNEO(1);
    } else if (moduleType.equals("sparkflex") || moduleType.equals("vortex") || moduleType.equals("sparkflex_vortex")) {
      return DCMotor.getNeoVortex(1);
    } else {
      throw new UnsupportedOperationException("Unsupported motor type: " + moduleType);
    }
  }

  public SwerveDrive getAsYAGSLDrive() {
    // FIXME: implement this
    return null;
  }

}
