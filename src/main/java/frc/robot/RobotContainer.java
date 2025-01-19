// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.helpers.LogitechX3D;

public class RobotContainer {

  static final class Constants {
    public static final int driverXboxPort = 0;
    public static final int operatorXboxPort = 1;
    public static final int driverFlightstickPort = 2;
    public static final double driverXboxDeadband = 0.1;
    public static final double driverFlightstickDeadband = 0.1;
    public static final double fastDriveScale = 1.0;
    public static final double slowDriveScale = 0.25;
  }

  CommandXboxController driverXboxController = new CommandXboxController(Constants.driverXboxPort);
  CommandXboxController operatorXboxController = new CommandXboxController(Constants.operatorXboxPort);
  LogitechX3D driverFlightstickController = new LogitechX3D(Constants.driverFlightstickPort);

  Drivetrain drivetrain = new Drivetrain();
  Arm arm = new Arm();

  void registerCommandsForUseInAutonomous() {
    // TODO: make more commands for auto to use
    Command doNothing = new InstantCommand();
    NamedCommands.registerCommand("doNothing", doNothing);
  }

  SendableChooser<Command> createDriverStationAutoChooser() {
    // TODO: return AutoBuilder.buildAutoChooser();
    return new SendableChooser<Command>();
  }

  void bindCommandsToDriverController() {

    Command slowFieldRelativeDriverControl = new RunCommand(
        () -> drivetrain.drive(
            getDriverForwardAxis() * Constants.slowDriveScale,
            getDriverSidewaysAxis() * Constants.slowDriveScale,
            getDriverRotationAxis() * Constants.slowDriveScale,
            true,
            false),
        drivetrain);

    Command recalibrateDriveTrain = new RunCommand(drivetrain::recalibrateDrivetrain, drivetrain);
    Command moveArmToMid = new RunCommand(arm::move_mid, arm);
    Command moveArmToAmp = new RunCommand(arm::move_amp, arm);
    Command moveArmToDefault = new RunCommand(arm::move_Default, arm);

    driverXboxController.leftBumper().whileTrue(slowFieldRelativeDriverControl);
    driverXboxController.back().whileTrue(recalibrateDriveTrain);

    driverFlightstickController.button(2).whileTrue(slowFieldRelativeDriverControl);
    driverFlightstickController.button(5).whileTrue(recalibrateDriveTrain);
    driverFlightstickController.button(10).whileTrue(moveArmToDefault);
    driverFlightstickController.button(11).whileTrue(moveArmToMid);
    driverFlightstickController.button(12).whileTrue(moveArmToAmp);

  }

  void bindCommandsToOperatorController() {

    final Command moveArmToMid = new RunCommand(() -> arm.move_mid(), arm);
    final Command moveArmToAmp = new RunCommand(() -> arm.move_amp(), arm);

    operatorXboxController.b().whileTrue(moveArmToMid);
    operatorXboxController.y().whileTrue(moveArmToAmp);

  }

  void ensureSubsystemsHaveDefaultCommands() {

    Command fastFieldRelativeDriverControl = new RunCommand(
        () -> drivetrain.drive(
            getDriverForwardAxis() * Constants.fastDriveScale,
            getDriverSidewaysAxis() * Constants.fastDriveScale,
            getDriverRotationAxis() * Constants.fastDriveScale,
            true,
            false),
        drivetrain);

    Command restTheArm = new RunCommand(arm::move_Default, arm);

    drivetrain.setDefaultCommand(fastFieldRelativeDriverControl);
    arm.setDefaultCommand(restTheArm);

  }

  private double getDriverForwardAxis() {
    return (MathUtil.applyDeadband(driverXboxController.getLeftY(), Constants.driverXboxDeadband) +
        MathUtil.applyDeadband(driverFlightstickController.getY(), Constants.driverFlightstickDeadband));
  }

  private double getDriverSidewaysAxis() {
    return (MathUtil.applyDeadband(driverXboxController.getLeftX(), Constants.driverXboxDeadband) +
        MathUtil.applyDeadband(driverFlightstickController.getX(), Constants.driverFlightstickDeadband));
  }

  private double getDriverRotationAxis() {
    return (MathUtil.applyDeadband(driverXboxController.getRightX(), Constants.driverXboxDeadband) +
        MathUtil.applyDeadband(driverFlightstickController.getTwist(), Constants.driverFlightstickDeadband));
  }
}