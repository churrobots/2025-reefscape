// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

public class RobotContainer {

  Drivetrain drivetrain = new Drivetrain();
  Intake intake = new Intake();
  Pipeshooter pipeshooter = new Pipeshooter();

  CommandXboxController driverXboxController = new CommandXboxController(Hardware.DriverStation.driverXboxPort);
  CommandXboxController operatorXboxController = new CommandXboxController(Hardware.DriverStation.operatorXboxPort);
  LogitechX3D driverFlightstickController = new LogitechX3D(Hardware.DriverStation.driverFlightstickPort);

  SendableChooser<Command> autoChooser;

  void bindCommandsForTeleop() {

    Command recalibrateDriveTrain = new RunCommand(() -> drivetrain.recalibrateDrivetrain(), drivetrain);
    Command bestIntake = new RunCommand(() -> intake.yoink(), intake);
    Command coralIntaker = new RunCommand(() -> pipeshooter.coralIntake(), pipeshooter);
    Command coralFeeder = new RunCommand(() -> pipeshooter.feedCoral(), pipeshooter);

    Command fastFieldRelativeDriverFlightstickControl = new RunCommand(
        () -> drivetrain.drive(new ChassisSpeeds(
            driverFlightstickController.getY() * Hardware.DriverStation.fastDriveScale,
            driverFlightstickController.getX() * Hardware.DriverStation.fastDriveScale,
            driverFlightstickController.getTwist() * Hardware.DriverStation.fastDriveScale)),
        drivetrain);

    Command slowFieldRelativeDriverFlightstickControl = new RunCommand(
        () -> drivetrain.drive(new ChassisSpeeds(
            driverFlightstickController.getY() * Hardware.DriverStation.slowDriveScale,
            driverFlightstickController.getX() * Hardware.DriverStation.slowDriveScale,
            driverFlightstickController.getTwist() * Hardware.DriverStation.slowDriveScale)),
        drivetrain);

    Command fastFieldRelativeDriverXboxControl = new RunCommand(
        () -> drivetrain.drive(new ChassisSpeeds(
            driverXboxController.getLeftY() * Hardware.DriverStation.fastDriveScale,
            driverXboxController.getLeftX() * Hardware.DriverStation.fastDriveScale,
            driverXboxController.getRightX() * Hardware.DriverStation.fastDriveScale)),
        drivetrain);

    Command slowFieldRelativeDriverXboxControl = new RunCommand(
        () -> drivetrain.drive(new ChassisSpeeds(
            driverXboxController.getLeftY() * Hardware.DriverStation.slowDriveScale,
            driverXboxController.getLeftX() * Hardware.DriverStation.slowDriveScale,
            driverXboxController.getRightX() * Hardware.DriverStation.slowDriveScale)),
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
    // TODO: use AutoBuilder.buildAutoChooser();
    autoChooser = new SendableChooser<Command>();
    SmartDashboard.putData(autoChooser);
    Elastic.enableDashboardToBeDownloadedFromRobotDeployDirectory();
  }

  Command readSelectedAutonomousCommand() {
    return autoChooser.getSelected();
  }

}