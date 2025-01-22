// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.churrolib.SimulationRegistry;
import frc.churrolib.RevMAXSwerveModule;
import frc.churrolib.RevMAXSwerveUtils;
import frc.robot.Hardware;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO(Controls): Support robot-relative driving so the Operator can use the live camera feed to position the robot for placing coral on the reef.
// Note: there is a robot-relative boolean on one of the drive APIs that can be used for this purpose.
// Steps:
// (1) add a command for robot-relative driving
// (2) design the user interaction -- does the driver press and hold a button to release driving control to the operator? What controller does the operator use? (Flightstick seems like a good fit).
// (3) map the joystick axes/buttons accordingly in RobotContainer.java

public class Drivetrain extends SubsystemBase {

  private static final class Constants {

    // Chassis configuration
    public static final double kTrackWidth = Hardware.RevMAXSwerveTemplate.kTrackWidth;
    public static final double kWheelBase = Hardware.RevMAXSwerveTemplate.kWheelBase;

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Hardware.RevMAXSwerveTemplate.kFrontLeftChassisAngularOffset;
    public static final double kFrontRightChassisAngularOffset = Hardware.RevMAXSwerveTemplate.kFrontRightChassisAngularOffset;
    public static final double kRearLeftChassisAngularOffset = Hardware.RevMAXSwerveTemplate.kRearLeftChassisAngularOffset;
    public static final double kRearRightChassisAngularOffset = Hardware.RevMAXSwerveTemplate.kRearRightChassisAngularOffset;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = Hardware.RevMAXSwerveTemplate.kMaxSpeedMetersPerSecond;
    public static final double kMaxAngularSpeed = Hardware.RevMAXSwerveTemplate.kMaxAngularSpeed;

    public static final double kDirectionSlewRate = Hardware.RevMAXSwerveTemplate.kDirectionSlewRate;
    public static final double kMagnitudeSlewRate = Hardware.RevMAXSwerveTemplate.kMagnitudeSlewRate;
    public static final double kRotationalSlewRate = Hardware.RevMAXSwerveTemplate.kRotationalSlewRate;
  }

  // Logging helpers.
  final StructArrayPublisher<SwerveModuleState> m_actualSwerveStatePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("ActualSwerveStates", SwerveModuleState.struct).publish();
  final StructArrayPublisher<SwerveModuleState> m_desiredSwerveStatePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("DesiredSwerveStates", SwerveModuleState.struct).publish();
  final Field2d m_fieldViz = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  // and preventing excessive swerve wheel wear.
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private final SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.kMagnitudeSlewRate);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.kRotationalSlewRate);

  private final RevMAXSwerveModule m_frontLeft = new RevMAXSwerveModule(
      Hardware.RevMAXSwerveTemplate.frontLeftDrivingMotorCAN,
      Hardware.RevMAXSwerveTemplate.frontLeftTurningMotorCAN, Constants.kFrontLeftChassisAngularOffset);

  private final RevMAXSwerveModule m_frontRight = new RevMAXSwerveModule(
      Hardware.RevMAXSwerveTemplate.frontRightDrivingMotorCAN,
      Hardware.RevMAXSwerveTemplate.frontRightTurningMotorCAN, Constants.kFrontRightChassisAngularOffset);

  private final RevMAXSwerveModule m_rearLeft = new RevMAXSwerveModule(
      Hardware.RevMAXSwerveTemplate.rearLeftDrivingMotorCAN,
      Hardware.RevMAXSwerveTemplate.rearLeftTurningMotorCAN, Constants.kRearLeftChassisAngularOffset);

  private final RevMAXSwerveModule m_rearRight = new RevMAXSwerveModule(
      Hardware.RevMAXSwerveTemplate.rearRightDrivingMotorCAN,
      Hardware.RevMAXSwerveTemplate.rearRightTurningMotorCAN, Constants.kRearRightChassisAngularOffset);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2));

  private final Pigeon2 m_gyro = new Pigeon2(Hardware.RevMAXSwerveTemplate.pigeonGyroCAN);
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      getGyroAngle(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d());

  public Drivetrain() {
    setDefaultCommand(new RunCommand(this::stop, this));
    SmartDashboard.putData("Field", m_fieldViz);
    SimulationRegistry.registerHardware(m_gyro);
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
        getGyroAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    SmartDashboard.putNumber("DrivetrainGyro", m_gyro.getRotation2d().getRadians());
    m_actualSwerveStatePublisher.set(getModuleStates());
    m_desiredSwerveStatePublisher.set(getDesiredModuleStates());
    m_fieldViz.getObject("Odometry").setPose(m_poseEstimator.getEstimatedPosition());
  }

  ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  SwerveModuleState[] getDesiredModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getDesiredState(),
        m_frontRight.getDesiredState(),
        m_rearLeft.getDesiredState(),
        m_rearRight.getDesiredState()
    };
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Resets the gyro as if the robot were facing away from you.
   * This is helpful for resetting field-oriented driving.
   */
  public void recalibrateDrivetrain() {
    double recalibratedAngleRadians = 0;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      recalibratedAngleRadians = Math.PI;
    }
    var currentPose = getPose();
    var currentTranslation = currentPose.getTranslation();
    var recalibratedRotation = new Rotation2d(recalibratedAngleRadians);
    var recalibratedPose = new Pose2d(currentTranslation, recalibratedRotation);
    resetPose(recalibratedPose);
  }

  void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  // Reference:
  // https://github.com/firebears-frc/FB2024/blob/main/src/main/java/frc/robot/subsystems/Bass.java#L284
  void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    }
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      xSpeed = -1 * xSpeed;
      ySpeed = -1 * ySpeed;
    }

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = RevMAXSwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = RevMAXSwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                              // checking
                                              // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = RevMAXSwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = RevMAXSwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * Constants.kMaxAngularSpeed;

    drive(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered), fieldRelative);

  }

  public void stop() {
    drive(0, 0, 0, true, true);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setXFormation() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble() % 360);
  }

}