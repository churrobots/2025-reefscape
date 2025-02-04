// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.churrolib.LogitechX3D;
import frc.churrolib.vendor.Elastic;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pipeshooter;
import frc.robot.subsystems.Drivetrain2;
import swervelib.SwerveInputStream;

// Need if using USB camera to roboRIO.
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.util.PixelFormat;

public class RobotContainer2 {

  Pipeshooter pipeshooter = new Pipeshooter();
  Drivetrain2 drivebase = new Drivetrain2(new File(Filesystem.getDeployDirectory(),
      "yagsl-configs/alpha"));

  void bindCommandsForTeleop() {

    CommandXboxController driverXbox = new CommandXboxController(Hardware.DriverStation.driverXboxPort);

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> driverXbox.getLeftY() * -1,
        () -> driverXbox.getLeftX() * -1)
        .withControllerRotationAxis(driverXbox::getRightX)
        .deadband(Hardware.DriverStation.driverXboxDeadband)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
        driverXbox::getRightY)
        .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
        .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX())
        .withControllerRotationAxis(() -> driverXbox.getRawAxis(
            2))
        .deadband(Hardware.DriverStation.driverXboxDeadband)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
        .withControllerHeadingAxis(() -> Math.sin(
            driverXbox.getRawAxis(
                2) *
                Math.PI)
            *
            (Math.PI *
                2),
            () -> Math.cos(
                driverXbox.getRawAxis(
                    2) *
                    Math.PI)
                *
                (Math.PI *
                    2))
        .headingWhile(true);

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
    // TODO: setup any camera feeds or other driver tools here

    // Elastic Dashboard setup notes:
    // Set Settings->Network->IP Address Mode to
    // "RoboRIO mDNS (roboRIO-###-FRC.local)"

    // TODO(Controls): create a JSON Elastic Dashboard layout to be stored on
    // the robot and copied to the Driver Station.

    ///////////////////////////// CAMERA SETUP ////////////////////////////////////

    // Camera Option 1: Microsoft Lifecam HD 3000 webcam plugged directly into
    // roboRIO over USB.
    //
    // Viewing in Elastic Dashboard:
    // Appears as CameraPublisher->"USB Camera 0".
    // Set Elastic CameraStream widget properties to FPS=30, Resolution=320x240,
    // Quality=0. Should get around 30fps.
    //
    // Notes:
    // - This camera has a hardware maximum of 30fps.
    // - There are some wavy effects on the image when moving quickly, potentially
    // due to rolling shutter?

    // UsbCamera frontCam = CameraServer.startAutomaticCapture();
    // frontCam.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
    // frontCam.setExposureManual(30);

    // Camera Option 2: Arducam OV9281 connected over USB to the OrangePi running
    // PhotonVision. OrangePi is connected over Ethernet to the robot router.
    // For now, power the OrangePi via an external USB C power brick.
    //
    // Configure camera via photonvision: http://photonvision.local:5800/#/dashboard
    // Camera Name="Churrovision".
    // Recommend resolution of either 320x240 @ 100Hz MJPEG , or 640x480 @ 100Hz
    // MJPEG
    //
    // Viewing in Elastic Dashboard:
    // Appears as CameraPublisher->"photonvision_Port_1184_Output_MJPEG_Server".
    // Set Elastic CameraStream FPS/Resolution to match whatever was configured on
    // the photonvision dashboard. At Quality=0, observed ~50fps for the 640x480,
    // ~100fps for the 320x240.
    //
    // No code needed for Option 2.

    ////////////////////////////////////////////////////////////////////////////////

    Elastic.enableDashboardToBeDownloadedFromRobotDeployDirectory();
    SmartDashboard.putString("Robot Name", Hardware.robotName);

  }

  Supplier<Command> bindCommandsForAutonomous() {

    // RobotConfig config;
    // try {
    // config = RobotConfig.fromGUISettings();
    // AutoBuilder.configure(
    // drivetrain::getPose, // Robot pose supplier
    // drivetrain::resetPose, // Method to reset odometry (will be called if your
    // auto has a starting pose)
    // drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
    // RELATIVE
    // (speeds, feedforwards) -> drivetrain.setRobotRelativeSpeeds(speeds), //
    // Method that will drive the robot given
    // // ROBOT RELATIVE ChassisSpeeds. Also
    // // optionally outputs individual module
    // // feedforwards
    // new PPHolonomicDriveController( // PPHolonomicController is the built in path
    // following controller for
    // // holonomic drive trains
    // new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    // new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    // ),
    // config, // The robot configuration
    // () -> {
    // // Boolean supplier that controls when the path will be mirrored for the red
    // // alliance
    // // This will flip the path being followed to the red side of the field.
    // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    // boolean isBlueAlliance = DriverStation.getAlliance().orElseGet(() ->
    // Alliance.Blue) == Alliance.Blue;
    // boolean shouldFlip = !isBlueAlliance;
    // return shouldFlip;
    // },
    // drivetrain // Reference to this subsystem to set requirements
    // );
    // } catch (Exception e) {
    // // Handle exception as needed
    // e.printStackTrace();
    // }

    // TODO: make real commands for auto to use
    NamedCommands.registerCommand("move1Beta", new InstantCommand());
    NamedCommands.registerCommand("move2Sigma", new InstantCommand());
    NamedCommands.registerCommand("move3Alpha", new InstantCommand());
    NamedCommands.registerCommand("waitForTeammates", new InstantCommand());
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    return autoChooser::getSelected;
  }

}
