// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

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
import frc.churrolib.CTRESingleFalconRollerSim;
import frc.churrolib.DeviceRegistry;
import frc.churrolib.GenericSwerveSim;
import frc.churrolib.RevMAXSwerveModuleSim;

/** Add your docs here. */
public class RobotSimulator {

  final CTRESingleFalconRollerSim m_shooterSim;

  final RevMAXSwerveModuleSim m_revTemplateSimFR;
  final RevMAXSwerveModuleSim m_revTemplateSimFL;
  final RevMAXSwerveModuleSim m_revTemplateSimRR;
  final RevMAXSwerveModuleSim m_revTemplateSimRL;
  final GenericSwerveSim m_swerveSim;

  final Mechanism2d m_vizRoller;
  final MechanismRoot2d m_vizAxle;
  final MechanismLigament2d m_vizWheels;
  final Field2d m_vizField;

  // Visualize this as a yellow box, so you can see it spinning.
  // https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html

  public RobotSimulator() {
    TalonFX coralMotor = DeviceRegistry.getTalonFX(Hardware.Shooter.falconMotorCAN);
    m_shooterSim = new CTRESingleFalconRollerSim(
        coralMotor, Hardware.Shooter.gearboxReduction, Hardware.Shooter.simMomentOfInertia);

    m_revTemplateSimFL = new RevMAXSwerveModuleSim(
        DeviceRegistry.getSparkMax(Hardware.RevMAXSwerveTemplate.frontLeftDrivingMotorCAN),
        Hardware.RevMAXSwerveTemplate.drivingMotorGearboxReduction,
        Hardware.RevMAXSwerveTemplate.drivingEncoderVelocityFactorInMetersPerSecond,
        DeviceRegistry.getSparkMax(Hardware.RevMAXSwerveTemplate.frontLeftTurningMotorCAN),
        Hardware.RevMAXSwerveTemplate.turningMotorGearboxReduction,
        Hardware.RevMAXSwerveTemplate.turningEncoderVelocityFactor,
        Hardware.RevMAXSwerveTemplate.kFrontLeftChassisAngularOffset);
    m_revTemplateSimFR = new RevMAXSwerveModuleSim(
        DeviceRegistry.getSparkMax(Hardware.RevMAXSwerveTemplate.frontRightDrivingMotorCAN),
        Hardware.RevMAXSwerveTemplate.drivingMotorGearboxReduction,
        Hardware.RevMAXSwerveTemplate.drivingEncoderVelocityFactorInMetersPerSecond,
        DeviceRegistry.getSparkMax(Hardware.RevMAXSwerveTemplate.frontRightTurningMotorCAN),
        Hardware.RevMAXSwerveTemplate.turningMotorGearboxReduction,
        Hardware.RevMAXSwerveTemplate.turningEncoderVelocityFactor,
        Hardware.RevMAXSwerveTemplate.kFrontRightChassisAngularOffset);
    m_revTemplateSimRL = new RevMAXSwerveModuleSim(
        DeviceRegistry.getSparkMax(Hardware.RevMAXSwerveTemplate.rearLeftDrivingMotorCAN),
        Hardware.RevMAXSwerveTemplate.drivingMotorGearboxReduction,
        Hardware.RevMAXSwerveTemplate.drivingEncoderVelocityFactorInMetersPerSecond,
        DeviceRegistry.getSparkMax(Hardware.RevMAXSwerveTemplate.rearLeftTurningMotorCAN),
        Hardware.RevMAXSwerveTemplate.turningMotorGearboxReduction,
        Hardware.RevMAXSwerveTemplate.turningEncoderVelocityFactor,
        Hardware.RevMAXSwerveTemplate.kRearLeftChassisAngularOffset);
    m_revTemplateSimRR = new RevMAXSwerveModuleSim(
        DeviceRegistry.getSparkMax(Hardware.RevMAXSwerveTemplate.rearRightDrivingMotorCAN),
        Hardware.RevMAXSwerveTemplate.drivingMotorGearboxReduction,
        Hardware.RevMAXSwerveTemplate.drivingEncoderVelocityFactorInMetersPerSecond,
        DeviceRegistry.getSparkMax(Hardware.RevMAXSwerveTemplate.rearRightTurningMotorCAN),
        Hardware.RevMAXSwerveTemplate.turningMotorGearboxReduction,
        Hardware.RevMAXSwerveTemplate.turningEncoderVelocityFactor,
        Hardware.RevMAXSwerveTemplate.kRearRightChassisAngularOffset);

    // Setup visualizations
    m_vizRoller = new Mechanism2d(1, 1);
    m_vizAxle = m_vizRoller.getRoot("Roller Axle", 0.5, 0.5);
    m_vizWheels = m_vizAxle
        .append(new MechanismLigament2d("Roller Wheels", 0.1, 0.0, 5.0, new Color8Bit(Color.kYellow)));
    // TODO: figure out where we should centralize the instantiation of a Field2d
    // (maybe sim has a separate Field2d anyway? then odometry can be selectively
    // placed on it?)
    m_vizField = (Field2d) SmartDashboard.getData("Field");
    SmartDashboard.putData("TestShooter", m_vizRoller);

    // TODO: decouple the viz from this one
    SwerveDriveKinematics simKinematics = new SwerveDriveKinematics(
        new Translation2d(Hardware.RevMAXSwerveTemplate.kWheelBase / 2, Hardware.RevMAXSwerveTemplate.kTrackWidth / 2),
        new Translation2d(Hardware.RevMAXSwerveTemplate.kWheelBase / 2, -Hardware.RevMAXSwerveTemplate.kTrackWidth / 2),
        new Translation2d(-Hardware.RevMAXSwerveTemplate.kWheelBase / 2, Hardware.RevMAXSwerveTemplate.kTrackWidth / 2),
        new Translation2d(-Hardware.RevMAXSwerveTemplate.kWheelBase / 2,
            -Hardware.RevMAXSwerveTemplate.kTrackWidth / 2));
    Supplier<ChassisSpeeds> chassisSpeedsSupplier = () -> {
      return simKinematics.toChassisSpeeds(new SwerveModuleState[] {
          m_revTemplateSimFL.getSimulatedState(),
          m_revTemplateSimFR.getSimulatedState(),
          m_revTemplateSimRL.getSimulatedState(),
          m_revTemplateSimRR.getSimulatedState()
      });
    };
    m_swerveSim = new GenericSwerveSim(
        DeviceRegistry.getPigeon2(Hardware.RevMAXSwerveTemplate.pigeonGyroCAN),
        chassisSpeedsSupplier,
        m_vizField);
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
    double speedReductionPercentageSoSpinningIsVisibleToHumanEye = 0.03;
    m_vizWheels.setAngle(
        m_vizWheels.getAngle()
            + Math.toDegrees(m_shooterSim.rollerOutputVelocityRPM())
                * timeDeltaInSeconds
                * speedReductionPercentageSoSpinningIsVisibleToHumanEye);
  }
}
