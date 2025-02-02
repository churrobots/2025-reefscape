// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.churrolib.simulation.CTRESingleFalconRollerSim;
import frc.churrolib.simulation.GenericSwerveSim;
import frc.churrolib.simulation.RevMAXSwerveModuleSim;
import frc.churrolib.simulation.SimulationRegistry;

public class RobotSimulator {

  final CTRESingleFalconRollerSim m_shooterSim;

  final RevMAXSwerveModuleSim m_revTemplateSimFR; // front right
  final RevMAXSwerveModuleSim m_revTemplateSimFL; // front left
  final RevMAXSwerveModuleSim m_revTemplateSimRR; // rear right
  final RevMAXSwerveModuleSim m_revTemplateSimRL; // rear left
  // Abstraction of the entire robot body given the activity of the above four
  // modules.
  final GenericSwerveSim m_swerveSim;

  final Mechanism2d m_vizRoller;
  final MechanismRoot2d m_vizAxle;
  final MechanismLigament2d m_vizWheels;
  final Field2d m_vizField;

  public RobotSimulator() {
    TalonFX coralMotor = SimulationRegistry.getTalonFX(Hardware.Shooter.falconMotorCAN);
    m_shooterSim = new CTRESingleFalconRollerSim(
        coralMotor, Hardware.Shooter.gearboxReduction, Hardware.Shooter.simMomentOfInertia);

    double moduleXOffsetAbsoluteValueInMeters = Hardware.DrivetrainWithTemplate.kTrackWidth / 2;
    double moduleYOffsetAbsoluteValueInMeters = Hardware.DrivetrainWithTemplate.kWheelBase / 2;
    m_revTemplateSimFL = new RevMAXSwerveModuleSim(
        SimulationRegistry.getSparkMax(Hardware.DrivetrainWithTemplate.frontLeftDrivingMotorCAN),
        Hardware.DrivetrainWithTemplate.drivingMotorGearboxReduction,
        Hardware.DrivetrainWithTemplate.drivingEncoderVelocityFactorInMetersPerSecond,
        SimulationRegistry.getSparkMax(Hardware.DrivetrainWithTemplate.frontLeftTurningMotorCAN),
        Hardware.DrivetrainWithTemplate.turningMotorGearboxReduction,
        Hardware.DrivetrainWithTemplate.turningEncoderVelocityFactor,
        Hardware.DrivetrainWithTemplate.kFrontLeftChassisAngularOffset,
        new Translation2d(moduleYOffsetAbsoluteValueInMeters, moduleXOffsetAbsoluteValueInMeters));
    m_revTemplateSimFR = new RevMAXSwerveModuleSim(
        SimulationRegistry.getSparkMax(Hardware.DrivetrainWithTemplate.frontRightDrivingMotorCAN),
        Hardware.DrivetrainWithTemplate.drivingMotorGearboxReduction,
        Hardware.DrivetrainWithTemplate.drivingEncoderVelocityFactorInMetersPerSecond,
        SimulationRegistry.getSparkMax(Hardware.DrivetrainWithTemplate.frontRightTurningMotorCAN),
        Hardware.DrivetrainWithTemplate.turningMotorGearboxReduction,
        Hardware.DrivetrainWithTemplate.turningEncoderVelocityFactor,
        Hardware.DrivetrainWithTemplate.kFrontRightChassisAngularOffset,
        new Translation2d(moduleXOffsetAbsoluteValueInMeters, -1 * moduleYOffsetAbsoluteValueInMeters));
    m_revTemplateSimRL = new RevMAXSwerveModuleSim(
        SimulationRegistry.getSparkMax(Hardware.DrivetrainWithTemplate.rearLeftDrivingMotorCAN),
        Hardware.DrivetrainWithTemplate.drivingMotorGearboxReduction,
        Hardware.DrivetrainWithTemplate.drivingEncoderVelocityFactorInMetersPerSecond,
        SimulationRegistry.getSparkMax(Hardware.DrivetrainWithTemplate.rearLeftTurningMotorCAN),
        Hardware.DrivetrainWithTemplate.turningMotorGearboxReduction,
        Hardware.DrivetrainWithTemplate.turningEncoderVelocityFactor,
        Hardware.DrivetrainWithTemplate.kRearLeftChassisAngularOffset,
        new Translation2d(-1 * moduleYOffsetAbsoluteValueInMeters, moduleXOffsetAbsoluteValueInMeters));
    m_revTemplateSimRR = new RevMAXSwerveModuleSim(
        SimulationRegistry.getSparkMax(Hardware.DrivetrainWithTemplate.rearRightDrivingMotorCAN),
        Hardware.DrivetrainWithTemplate.drivingMotorGearboxReduction,
        Hardware.DrivetrainWithTemplate.drivingEncoderVelocityFactorInMetersPerSecond,
        SimulationRegistry.getSparkMax(Hardware.DrivetrainWithTemplate.rearRightTurningMotorCAN),
        Hardware.DrivetrainWithTemplate.turningMotorGearboxReduction,
        Hardware.DrivetrainWithTemplate.turningEncoderVelocityFactor,
        Hardware.DrivetrainWithTemplate.kRearRightChassisAngularOffset,
        new Translation2d(-1 * moduleYOffsetAbsoluteValueInMeters, -1 * moduleXOffsetAbsoluteValueInMeters));

    // Setup visualizations
    // Visualize this as a yellow box, so you can see it spinning.
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
    m_vizRoller = new Mechanism2d(1, 1);
    m_vizAxle = m_vizRoller.getRoot("Roller Axle", 0.5, 0.5);
    m_vizWheels = m_vizAxle
        .append(new MechanismLigament2d("Roller Wheels", 0.1, 0.0, 5.0, new Color8Bit(Color.kYellow)));
    // TODO: figure out where we should centralize the instantiation of a Field2d
    // (maybe sim has a separate Field2d anyway? then odometry can be selectively
    // placed on it?)
    m_vizField = (Field2d) SmartDashboard.getData("Field");
    SmartDashboard.putData("TestShooter", m_vizRoller);

    // Create the all-in-one swerve sim (it has viz built-in)
    // Specify the location of each of the four swerve modules on the drivebase
    // rectange.
    //
    // TODO: decouple the viz from this one?
    // TODO: maybe it's okay for each Sim object to have a "rough viz" and then
    // expose methods for the main simulation to re-render in a different way and
    // link up sub-assemblies?
    SwerveDriveKinematics simKinematics = new SwerveDriveKinematics(
        new Translation2d(Hardware.DrivetrainWithTemplate.kWheelBase / 2,
            Hardware.DrivetrainWithTemplate.kTrackWidth / 2),
        new Translation2d(Hardware.DrivetrainWithTemplate.kWheelBase / 2,
            -Hardware.DrivetrainWithTemplate.kTrackWidth / 2),
        new Translation2d(-Hardware.DrivetrainWithTemplate.kWheelBase / 2,
            Hardware.DrivetrainWithTemplate.kTrackWidth / 2),
        new Translation2d(-Hardware.DrivetrainWithTemplate.kWheelBase / 2,
            -Hardware.DrivetrainWithTemplate.kTrackWidth / 2));

    // Given the positions and the activity of each of the four modules, determine
    // what the chassis isdoing as a whole.
    Supplier<ChassisSpeeds> chassisSpeedsSupplier = () -> {
      return simKinematics.toChassisSpeeds(new SwerveModuleState[] {
          m_revTemplateSimFL.getSimulatedState(),
          m_revTemplateSimFR.getSimulatedState(),
          m_revTemplateSimRL.getSimulatedState(),
          m_revTemplateSimRR.getSimulatedState()
      });
    };
    m_swerveSim = new GenericSwerveSim(
        SimulationRegistry.getPigeon2(Hardware.DrivetrainWithTemplate.pigeonGyroCAN),
        chassisSpeedsSupplier);
  }

  public void iterate(double timeDeltaInSeconds) {
    m_shooterSim.iterate(timeDeltaInSeconds);
    m_revTemplateSimFL.iterate(timeDeltaInSeconds);
    m_revTemplateSimFR.iterate(timeDeltaInSeconds);
    m_revTemplateSimRL.iterate(timeDeltaInSeconds);
    m_revTemplateSimRR.iterate(timeDeltaInSeconds);
    m_swerveSim.iterate(timeDeltaInSeconds);
  }

  public void render(double timeDeltaInSeconds) {
    // Update viz based on sim
    // TODO: try to use color to indicate speed, the "scaling" really messes with
    // the low speeds, and we really just want to indicate when the speed is so fast
    // that aliasing will occur. maybe scale only above a certain velocity?
    double speedReductionPercentageSoSpinningIsVisibleToHumanEye = 0.03;
    m_vizWheels.setAngle(
        m_vizWheels.getAngle()
            + Math.toDegrees(m_shooterSim.rollerOutputVelocityRPM())
                * timeDeltaInSeconds
                * speedReductionPercentageSoSpinningIsVisibleToHumanEye);

    // Show where the template drive thinks things are.
    // If we're using YAGSL, that simulation will already be using setRobotPose(),
    // so we need to identify this pose as a different pose (TemplateRobotPose) to
    // distinguish it in the simulation GUI.
    Pose2d robotPose = m_swerveSim.getRobotPose();
    Pose2d[] modulePoses = {
        m_revTemplateSimFL.getModulePose(robotPose),
        m_revTemplateSimFR.getModulePose(robotPose),
        m_revTemplateSimRL.getModulePose(robotPose),
        m_revTemplateSimRR.getModulePose(robotPose),
    };
    if (Hardware.Drivetrain.useYAGSL) {
      m_vizField.getObject("TemplateRobotPose").setPose(robotPose);
      m_vizField.getObject("TemplateRobotXModules").setPoses(modulePoses);
    } else {
      m_vizField.setRobotPose(robotPose);
      m_vizField.getObject("XModules").setPoses(modulePoses);
    }

  }
}
