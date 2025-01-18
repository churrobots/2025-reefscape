// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot.subsystems;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.wpilibj2.command.Command;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.churrolib.ChurroSim;
import frc.churrolib.GenericSwerveSim;
import frc.robot.CANMapping;
import frc.robot.helpers.RevMAXSwerveModule;
import frc.robot.helpers.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drivetrain extends SubsystemBase {

  private static final class Constants {

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    public static final double kWheelBase = Units.inchesToMeters(23.5);

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = Math.PI / 2;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 6.04;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 3.6; // radians per second
    public static final double kMagnitudeSlewRate = 4.5; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 6; // percent per second (1 = 100%)
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

  // private final RevMAXSwerveModule m_frontLeft = new
  // RevMAXSwerveModule(CANMapping.frontLeftDrivingMotor,
  // CANMapping.frontLeftTurningMotor, Constants.kFrontLeftChassisAngularOffset);

  // private final RevMAXSwerveModule m_frontRight = new
  // RevMAXSwerveModule(CANMapping.frontRightDrivingMotor,
  // CANMapping.frontRightTurningMotor,
  // Constants.kFrontRightChassisAngularOffset);

  // private final RevMAXSwerveModule m_rearLeft = new
  // RevMAXSwerveModule(CANMapping.rearLeftDrivingMotor,
  // CANMapping.rearLeftTurningMotor, Constants.kRearLeftChassisAngularOffset);

  // private final RevMAXSwerveModule m_rearRight = new
  // RevMAXSwerveModule(CANMapping.rearRightDrivingMotor,
  // CANMapping.rearRightTurningMotor, Constants.kRearRightChassisAngularOffset);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2));

  private final Pigeon2 m_gyro = new Pigeon2(CANMapping.gyroSensor);
  // private final SwerveDrivePoseEstimator m_poseEstimator = new
  // SwerveDrivePoseEstimator(
  // m_kinematics,
  // getGyroAngle(),
  // new SwerveModulePosition[] {
  // m_frontLeft.getPosition(),
  // m_frontRight.getPosition(),
  // m_rearLeft.getPosition(),
  // m_rearRight.getPosition()
  // },
  // new Pose2d());

  // private final GenericSwerveSim m_sim;

  // YAGSL Swerve
  private final SwerveDrive m_swerveDrive;

  public Drivetrain() {
    SmartDashboard.putData("Field", m_fieldViz);
    // m_sim = new GenericSwerveSim(m_gyro, this::getRobotRelativeSpeeds,
    // m_fieldViz);
    // ChurroSim.register(m_sim);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    File m_swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve/canelo");
    try {
      m_swerveDrive = new SwerveParser(m_swerveJsonDirectory).createSwerveDrive(Constants.kMaxSpeedMetersPerSecond,
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    m_swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                               // angle.
    m_swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation
                                              // for simulations since it causes discrepancies not seen in real life.
    m_swerveDrive.setAngularVelocityCompensation(true,
        true,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
              // coefficient of 0.1.
    m_swerveDrive.setModuleEncoderAutoSynchronize(false,
        1); // Enable if you want to resynchronize your absolute encoders and motor encoders
            // periodically when they are not moving.
  }

  @Override
  public void periodic() {
    // m_poseEstimator.update(
    // getGyroAngle(),
    // new SwerveModulePosition[] {
    // m_frontLeft.getPosition(),
    // m_frontRight.getPosition(),
    // m_rearLeft.getPosition(),
    // m_rearRight.getPosition()
    // });
    SmartDashboard.putNumber("DrivetrainGyro", m_gyro.getRotation2d().getRadians());
    // m_actualSwerveStatePublisher.set(getModuleStates());
    // m_desiredSwerveStatePublisher.set(getDesiredModuleStates());
    // m_fieldViz.getObject("Odometry").setPose(m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {
  }

  // ChassisSpeeds getRobotRelativeSpeeds() {
  // return m_kinematics.toChassisSpeeds(getModuleStates());
  // }

  // SwerveModuleState[] getModuleStates() {
  // return new SwerveModuleState[] {
  // m_frontLeft.getState(),
  // m_frontRight.getState(),
  // m_rearLeft.getState(),
  // m_rearRight.getState()
  // };
  // }

  // SwerveModuleState[] getDesiredModuleStates() {
  // return new SwerveModuleState[] {
  // m_frontLeft.getDesiredState(),
  // m_frontRight.getDesiredState(),
  // m_rearLeft.getDesiredState(),
  // m_rearRight.getDesiredState()
  // };
  // }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  // Pose2d getPose() {
  // return m_poseEstimator.getEstimatedPosition();
  // }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  // void resetPose(Pose2d pose) {
  // m_poseEstimator.resetPosition(
  // getGyroAngle(),
  // new SwerveModulePosition[] {
  // m_frontLeft.getPosition(),
  // m_frontRight.getPosition(),
  // m_rearLeft.getPosition(),
  // m_rearRight.getPosition()
  // },
  // pose);
  // }

  /**
   * Resets the gyro as if the robot were facing away from you.
   * This is helpful for resetting field-oriented driving.
   */
  // public void recalibrateDrivetrain() {
  // double recalibratedAngleRadians = 0;
  // var alliance = DriverStation.getAlliance();
  // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
  // recalibratedAngleRadians = Math.PI;
  // }
  // var currentPose = getPose();
  // var currentTranslation = currentPose.getTranslation();
  // var recalibratedRotation = new Rotation2d(recalibratedAngleRadians);
  // var recalibratedPose = new Pose2d(currentTranslation, recalibratedRotation);
  // resetPose(recalibratedPose);
  // }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command recalibrateDrivetrain() {
    return run(() -> Arrays.asList(m_swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }
  // void driveRobotRelative(ChassisSpeeds speeds) {
  // drive(speeds, false);
  // }

  // YAGSL Reference: https://docs.yagsl.com/configuring-yagsl/code-setup
  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    m_swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      m_swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    m_swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return m_swerveDrive.kinematics;
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          m_swerveDrive.getOdometryHeading().getRadians(),
          m_swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      m_swerveDrive.drive(new Translation2d(translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()),
          angularRotationX.getAsDouble() * m_swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  public void stop() {
    drive(new ChassisSpeeds());

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  // public void setXFormation() {
  // m_frontLeft.setDesiredState(new SwerveModuleState(0,
  // Rotation2d.fromDegrees(45)));
  // m_frontRight.setDesiredState(new SwerveModuleState(0,
  // Rotation2d.fromDegrees(-45)));
  // m_rearLeft.setDesiredState(new SwerveModuleState(0,
  // Rotation2d.fromDegrees(-45)));
  // m_rearRight.setDesiredState(new SwerveModuleState(0,
  // Rotation2d.fromDegrees(45)));
  // }

  // /**
  // * Sets the swerve ModuleStates.
  // *
  // * @param desiredStates The desired SwerveModule states.
  // */
  // void setModuleStates(SwerveModuleState[] desiredStates) {
  // SwerveDriveKinematics.desaturateWheelSpeeds(
  // desiredStates, Constants.kMaxSpeedMetersPerSecond);
  // m_frontLeft.setDesiredState(desiredStates[0]);
  // m_frontRight.setDesiredState(desiredStates[1]);
  // m_rearLeft.setDesiredState(desiredStates[2]);
  // m_rearRight.setDesiredState(desiredStates[3]);
  // }

  Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble() % 360);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return m_swerveDrive.getPose();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to
   * resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }
}