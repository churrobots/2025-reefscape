// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import swervelib.SwerveDrive;

public class Drivetrain extends SubsystemBase {

  final DrivetrainWithTemplate m_drivetrainWithTemplate;
  final DrivetrainWithYAGSL m_drivetrainWithYAGSL;

  public Drivetrain() {
    if (Hardware.Drivetrain.useYAGSL) {
      SmartDashboard.putString("DrivetrainType", "YAGSL");
      m_drivetrainWithYAGSL = new DrivetrainWithYAGSL();
      m_drivetrainWithTemplate = null;
    } else {
      SmartDashboard.putString("DrivetrainType", "Template");
      m_drivetrainWithTemplate = new DrivetrainWithTemplate();
      m_drivetrainWithYAGSL = null;
    }
  }

  public void setDefaultCommand(Command command) {
    if (Hardware.Drivetrain.useYAGSL) {
      m_drivetrainWithYAGSL.setDefaultCommand(command);
    } else {
      m_drivetrainWithTemplate.setDefaultCommand(command);
    }
  }

  public void recalibrateDrivetrain() {
    if (Hardware.Drivetrain.useYAGSL) {
      m_drivetrainWithYAGSL.recalibrateDrivetrain();
    } else {
      m_drivetrainWithTemplate.recalibrateDrivetrain();
    }
  }

  public Command createFieldRelativeDriveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    if (Hardware.Drivetrain.useYAGSL) {
      return m_drivetrainWithYAGSL.createFieldRelativeDriveCommand(
          translationX, translationY, angularRotationX);
    } else {
      return m_drivetrainWithTemplate.createFieldRelativeDriveCommand(
          translationX, translationY, angularRotationX);
    }
  }

  public Command createRobotRelativeDriveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    if (Hardware.Drivetrain.useYAGSL) {
      return m_drivetrainWithYAGSL.createRobotRelativeDriveCommand(
          translationX, translationY, angularRotationX);
    } else {
      return m_drivetrainWithTemplate.createRobotRelativeDriveCommand(
          translationX, translationY, angularRotationX);
    }
  }

  public Pose2d getPose() {
    if (Hardware.Drivetrain.useYAGSL) {
      return m_drivetrainWithYAGSL.getPose();
    } else {
      return m_drivetrainWithTemplate.getPose();
    }
  }

  public void resetPose(Pose2d newPose) {
    if (Hardware.Drivetrain.useYAGSL) {
      m_drivetrainWithYAGSL.resetPose(newPose);
    } else {
      m_drivetrainWithTemplate.resetPose(newPose);
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    if (Hardware.Drivetrain.useYAGSL) {
      return m_drivetrainWithYAGSL.getRobotRelativeSpeeds();
    } else {
      return m_drivetrainWithTemplate.getRobotRelativeSpeeds();
    }
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
    if (Hardware.Drivetrain.useYAGSL) {
      m_drivetrainWithYAGSL.setRobotRelativeSpeeds(speeds);
    } else {
      m_drivetrainWithTemplate.driveRobotRelative(speeds);
    }
  }

  public SwerveDriveKinematics getKinematics() {
    if (Hardware.Drivetrain.useYAGSL) {
      return m_drivetrainWithYAGSL.getKinematics();
    } else {
      return m_drivetrainWithTemplate.getKinematics();
    }
  }

  public void drive(
      ChassisSpeeds robotRelativeVelocity, SwerveModuleState[] states, Force[] feedforwardForces) {
    if (Hardware.Drivetrain.useYAGSL) {
      m_drivetrainWithYAGSL.drive(robotRelativeVelocity, states, feedforwardForces);
    } else {
      m_drivetrainWithTemplate.drive(robotRelativeVelocity, states, feedforwardForces);
    }
  }
}
