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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.churrolib.UniversalRobotProperties;
import frc.churrolib.HardwareRegistry;
import frc.robot.Hardware;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

// TODO(Controls): Support robot-relative driving so the Operator can use the live camera feed to position the robot for placing coral on the reef.
// Note: there is a robot-relative boolean on one of the drive APIs that can be used for this purpose.
// Steps:
// (1) add a command for robot-relative driving
// (2) design the user interaction -- does the driver press and hold a button to release driving control to the operator? What controller does the operator use? (Flightstick seems like a good fit).
// (3) map the joystick axes/buttons accordingly in RobotContainer.java

public class Drivetrain extends SubsystemBase {

  // Logging helpers.
  final StructArrayPublisher<SwerveModuleState> m_actualSwerveStatePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("ActualSwerveStates", SwerveModuleState.struct).publish();
  final StructArrayPublisher<SwerveModuleState> m_desiredSwerveStatePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("DesiredSwerveStates", SwerveModuleState.struct).publish();
  final StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("YagslPose", Pose2d.struct).publish();
  final Field2d m_fieldViz = new Field2d();

  // YAGSL Swerve
  private final SwerveDrive m_swerveDrive;
  private final File m_swerveJsonDirectory;
  private Vision m_vision;

  public Drivetrain() {
    // TODO: Record logging
    // DataLogManager.start();
    // DriverStation.startDataLog(DataLogManager.getLog());

    setDefaultCommand(new RunCommand(this::stop, this));
    SmartDashboard.putData("Field", m_fieldViz);
    if (Hardware.Diagnostics.debugTelemetry == true) {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    } else {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;
    }

    m_swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),
        Hardware.Drivetrain.swerveConfigDeployPath);
    try {
      m_swerveDrive = new SwerveParser(
          m_swerveJsonDirectory).createSwerveDrive(
              Hardware.Drivetrain.maxSpeedMetersPerSecond,
              new Pose2d(new Translation2d(Meter.of(1),
                  Meter.of(4)),
                  Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    HardwareRegistry.registerHardware(m_swerveDrive.getGyro().getIMU());
    HardwareRegistry.registerHardware(m_swerveDrive.getModules()[0].getDriveMotor().getMotor());
    HardwareRegistry.registerHardware(m_swerveDrive.getModules()[0].getAngleMotor().getMotor());
    HardwareRegistry.registerHardware(m_swerveDrive.getModules()[1].getDriveMotor().getMotor());
    HardwareRegistry.registerHardware(m_swerveDrive.getModules()[1].getAngleMotor().getMotor());
    HardwareRegistry.registerHardware(m_swerveDrive.getModules()[2].getDriveMotor().getMotor());
    HardwareRegistry.registerHardware(m_swerveDrive.getModules()[2].getAngleMotor().getMotor());
    HardwareRegistry.registerHardware(m_swerveDrive.getModules()[3].getDriveMotor().getMotor());
    HardwareRegistry.registerHardware(m_swerveDrive.getModules()[3].getAngleMotor().getMotor());

    m_swerveDrive.setHeadingCorrection(false);
    m_swerveDrive.setCosineCompensator(false);
    // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    m_swerveDrive.setAngularVelocityCompensation(true, true,
        0.1);

    // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    m_swerveDrive.setModuleEncoderAutoSynchronize(false,
        1);

    // TODO: this made it drive weird
    // m_swerveDrive.setChassisDiscretization(true, 0.02);

    // Setup vision
    if (Hardware.Vision.isEnabled) {
      m_vision = new Vision(m_swerveDrive::getPose, m_swerveDrive.field);
    }

  }

  public SendableChooser<Command> createPathPlannerDropdown() {
    RobotConfig config;
    try {
      // UniversalRobotProperties robotProperties = new
      // UniversalRobotProperties(m_swerveJsonDirectory,
      // Hardware.Drivetrain.maxSpeedMetersPerSecond, Hardware.Drivetrain.robotMOI);
      // config = robotProperties.getAsPathPlannerConfig();
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> {
            this.drive(speeds,
                this.getKinematics().toSwerveModuleStates(speeds),
                feedforwards.linearForces());
          },
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants (used to be 5.0)
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants (used to be 5.0)
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            boolean isBlueAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue;
            boolean shouldFlip = !isBlueAlliance;
            return shouldFlip;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    return AutoBuilder.buildAutoChooser();
  }

  @Override
  public void periodic() {
    if (Hardware.Vision.isEnabled) {
      m_vision.updatePoseEstimation(m_swerveDrive);
      m_swerveDrive.updateOdometry();
    }
    m_posePublisher.set(getPose());
    m_actualSwerveStatePublisher.set(getModuleStates());
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command recalibrateSwerveModules() {
    return run(() -> Arrays.asList(m_swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }

  public Command recalibrateDrivetrain() {
    return run(() -> {
      boolean isBlueAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue;
      Pose2d currentPose = m_swerveDrive.getPose();
      if (isBlueAlliance) {
        m_swerveDrive.resetOdometry(new Pose2d(
            currentPose.getX(),
            currentPose.getY(),
            Rotation2d.fromDegrees(0)));
      } else {
        m_swerveDrive.resetOdometry(new Pose2d(
            currentPose.getX(),
            currentPose.getY(),
            Rotation2d.fromDegrees(180)));

      }
    });
  }

  void driveRobotRelative(ChassisSpeeds speeds) {
    m_swerveDrive.drive(speeds);
  }

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

  public void drive(
      ChassisSpeeds robotRelativeVelocity, SwerveModuleState[] states, Force[] feedforwardForces) {
    m_swerveDrive.drive(robotRelativeVelocity, states, feedforwardForces);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return m_swerveDrive.kinematics;
  }

  public SwerveDrive getSwerveDrive() {
    return m_swerveDrive;
  }

  SwerveModuleState[] getModuleStates() {
    return m_swerveDrive.getStates();
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

      // TODO: is this artificially limiting to 80%?
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
  public Command createFieldRelativeDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
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

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command createRobotRelativeDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      m_swerveDrive.drive(new Translation2d(translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()),
          angularRotationX.getAsDouble() * m_swerveDrive.getMaximumChassisAngularVelocity(),
          false,
          false);
    });
  }

  public void resetPose(Pose2d newPose) {
    m_swerveDrive.resetOdometry(newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_swerveDrive.getRobotVelocity();
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
    m_swerveDrive.setChassisSpeeds(speeds);
  }

  public void stop() {
    drive(new ChassisSpeeds());
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