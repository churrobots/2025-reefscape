// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.churrolib.Elastic;
import frc.churrolib.LogitechX3D;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pipeshooter;

// Need if using USB camera to roboRIO.
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.util.PixelFormat;

public class RobotContainer {

  Drivetrain drivetrain = new Drivetrain();
  Intake intake = new Intake();
  Pipeshooter pipeshooter = new Pipeshooter();

  CommandXboxController driverXboxController = new CommandXboxController(Hardware.DriverStation.driverXboxPort);
  CommandXboxController operatorXboxController = new CommandXboxController(Hardware.DriverStation.operatorXboxPort);
  LogitechX3D driverFlightstickController = new LogitechX3D(Hardware.DriverStation.driverFlightstickPort);

  SendableChooser<Command> autoChooser;

  void bindCommandsForTeleop() {

    // TODO: confirm WASD is simulating joystick axes pos/neg directions correctly
    // TODO: figure out how sim handles the initial pose, and recalibrated poses
    Command recalibrateDriveTrain = new RunCommand(() -> drivetrain.recalibrateDrivetrain(), drivetrain);
    Command bestIntake = new RunCommand(() -> intake.yoink(), intake);
    Command coralIntaker = new RunCommand(() -> pipeshooter.coralIntake(), pipeshooter);
    Command coralFeeder = new RunCommand(() -> pipeshooter.feedCoral(), pipeshooter);

    Command fastFieldRelativeDriverFlightstickControl = new RunCommand(
        () -> drivetrain.drive(
            driverFlightstickController.getY() * Hardware.DriverStation.fastDriveScale,
            driverFlightstickController.getX() * Hardware.DriverStation.fastDriveScale,
            driverFlightstickController.getTwist() * Hardware.DriverStation.fastDriveScale,
            true,
            false),
        drivetrain);

    Command slowFieldRelativeDriverFlightstickControl = new RunCommand(
        () -> drivetrain.drive(
            driverFlightstickController.getY() * Hardware.DriverStation.slowDriveScale,
            driverFlightstickController.getX() * Hardware.DriverStation.slowDriveScale,
            driverFlightstickController.getTwist() * Hardware.DriverStation.slowDriveScale,
            true,
            false),
        drivetrain);

    Command fastFieldRelativeDriverXboxControl = new RunCommand(
        () -> drivetrain.drive(
            driverXboxController.getLeftY() * Hardware.DriverStation.fastDriveScale,
            driverXboxController.getLeftX() * Hardware.DriverStation.fastDriveScale,
            driverXboxController.getRightX() * Hardware.DriverStation.fastDriveScale,
            true,
            false),
        drivetrain);

    Command slowFieldRelativeDriverXboxControl = new RunCommand(
        () -> drivetrain.drive(
            driverXboxController.getLeftY() * Hardware.DriverStation.slowDriveScale,
            driverXboxController.getLeftX() * Hardware.DriverStation.slowDriveScale,
            driverXboxController.getRightX() * Hardware.DriverStation.slowDriveScale,
            true,
            false),
        drivetrain);

    if (Hardware.DriverStation.driverUsesFlightstick) {

      drivetrain.setDefaultCommand(fastFieldRelativeDriverFlightstickControl);
      driverFlightstickController.button(2).whileTrue(slowFieldRelativeDriverFlightstickControl);
      driverFlightstickController.button(5).whileTrue(recalibrateDriveTrain);
      driverFlightstickController.button(7).whileTrue(bestIntake);
      driverFlightstickController.button(8).whileTrue(coralIntaker);
      driverFlightstickController.button(9).whileTrue(coralFeeder);

    } else {

      drivetrain.setDefaultCommand(fastFieldRelativeDriverXboxControl);
      driverXboxController.leftBumper().whileTrue(slowFieldRelativeDriverXboxControl);
      driverXboxController.back().whileTrue(recalibrateDriveTrain);
      driverXboxController.a().whileTrue(coralIntaker);
      driverXboxController.b().whileTrue(coralFeeder);
    }

    operatorXboxController.b().whileTrue(coralFeeder);

  }

  void bindCommandsForAutonomous() {
    // TODO: make real commands for auto to use
    Command doNothing = new InstantCommand();
    NamedCommands.registerCommand("doNothing", doNothing);
  }

  void setupDriverStationDashboard() {
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

    // Allow the driver to select which autonomous to run via the SmartDashboard.
    // TODO: use AutoBuilder.buildAutoChooser();
    autoChooser = new SendableChooser<Command>();
    SmartDashboard.putData(autoChooser);
    Elastic.enableDashboardToBeDownloadedFromRobotDeployDirectory();
  }

  Command readSelectedAutonomousCommand() {
    return autoChooser.getSelected();
  }

}