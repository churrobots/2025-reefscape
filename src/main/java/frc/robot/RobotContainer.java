// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.churrolib.LogitechX3D;
import frc.churrolib.vendor.Elastic;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pipeshooter;

// Need if using USB camera to roboRIO.
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;

public class RobotContainer {

  Pipeshooter pipeshooter = new Pipeshooter();
  Drivetrain drivetrain = new Drivetrain();

  void bindCommandsForTeleop() {

    CommandXboxController driverXboxController = new CommandXboxController(Hardware.DriverStation.driverXboxPort);
    CommandXboxController operatorXboxController = new CommandXboxController(Hardware.DriverStation.operatorXboxPort);

    // TODO: confirm WASD is simulating joystick axes pos/neg directions correctly
    // TODO: figure out how sim handles the initial pose, and recalibrated poses
    Command recalibrateDriveTrain = new RunCommand(() -> drivetrain.recalibrateDrivetrain(), drivetrain);

    DoubleSupplier allianceRelativeFactor = () -> {
      boolean isRedAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Red) == Alliance.Red;
      double yagslInvert = Hardware.Drivetrain.useYAGSL ? -1 : 1;
      if (isRedAlliance) {
        return 1.0 * yagslInvert;
      } else {
        return -1.0 * yagslInvert;
      }
    };
    double xboxDeadband = Hardware.DriverStation.driverXboxDeadband;

    Command fastFieldRelativeDriverXboxControl = drivetrain.createFieldRelativeDriveCommand(
        () -> allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(driverXboxController.getLeftY(), xboxDeadband)
            * Hardware.DriverStation.fastDriveScale,
        () -> allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(driverXboxController.getLeftX(), xboxDeadband)
            * Hardware.DriverStation.fastDriveScale,
        () -> -1 * MathUtil.applyDeadband(driverXboxController.getRightX(), xboxDeadband)
            * Hardware.DriverStation.fastDriveScale);

    Command slowFieldRelativeDriverXboxControl = drivetrain.createFieldRelativeDriveCommand(
        () -> allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(driverXboxController.getLeftY(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale,
        () -> allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(driverXboxController.getLeftX(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale,
        () -> -1 * MathUtil.applyDeadband(driverXboxController.getRightX(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale);

    Command slowRobotRelativeOperatorXboxControl = drivetrain.createRobotRelativeDriveCommand(
        () -> allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(operatorXboxController.getLeftY(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale
            * -1,
        () -> allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(operatorXboxController.getLeftX(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale
            * -1,
        () -> -1 * MathUtil.applyDeadband(operatorXboxController.getRightX(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale);

    drivetrain.setDefaultCommand(fastFieldRelativeDriverXboxControl);
    driverXboxController.leftBumper().whileTrue(slowFieldRelativeDriverXboxControl);
    driverXboxController.back().whileTrue(recalibrateDriveTrain);





    ///////////////////////////// CAMERA SETUP ////////////////////////////////////

    // TODO: setup any camera feeds or other driver tools here

    // Elastic Dashboard setup notes:
    // Set Settings->Network->IP Address Mode to
    // "RoboRIO mDNS (roboRIO-###-FRC.local)"

    // TODO(Controls): create a JSON Elastic Dashboard layout to be stored on
    // the robot and copied to the Driver Station.

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

    if (RobotBase.isReal()) {
      // UsbCamera frontCam = CameraServer.startAutomaticCapture();
      // frontCam.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
      // frontCam.setExposureManual(55);

      Thread m_visionThread = new Thread(
          () -> {
            // Get the UsbCamera from CameraServer
            UsbCamera camera = CameraServer.startAutomaticCapture();
            // Set the resolution
            camera.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);

            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();
            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);

            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat = new Mat();

            // This cannot be 'true'. The program will never exit if it is. This
            // lets the robot stop this thread when restarting robot code or
            // deploying.
            while (!Thread.interrupted()) {
              // Tell the CvSink to grab a frame from the camera and put it
              // in the source mat. If there is an error notify the output.
              if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
              }
              int centerX = mat.cols() / 2;
              int centerY = mat.rows() / 2;

              // Crosshair vertical line
              Imgproc.line(mat,
                  new org.opencv.core.Point(centerX, 0),
                  new org.opencv.core.Point(centerX, mat.rows()),
                  new Scalar(0, 255, 0), 2);

              // Crosshair horizontal line
              Imgproc.line(mat,
                  new org.opencv.core.Point(0, centerY),
                  new org.opencv.core.Point(mat.cols(), centerY),
                  new Scalar(0, 255, 0), 2);
              // Put a rectangle on the image
              // Imgproc.rectangle(
              // mat, new Point(10, 10), new Point(40, 40), new Scalar(255, 255, 255), 5);
              // Give the output stream a new image to display
              outputStream.putFrame(mat);
            }
          });
      m_visionThread.setDaemon(true);
      m_visionThread.start();
    }

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
    // TODO: make real commands for auto to use
    NamedCommands.registerCommand("doNothing", new InstantCommand());
    // TODO: use AutoBuilder.buildAutoChooser() instead
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    SmartDashboard.putData(autoChooser);
    return autoChooser::getSelected;
  }

}