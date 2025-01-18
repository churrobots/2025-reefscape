// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.helpers.LogitechX3D;

public class RobotContainer {

  static final class Constants {
    public static final int kDriverXboxPort = 0;
    public static final int kOperatorXboxPort = 1;
    public static final int kDriverX3DPort = 2;
    public static final double kDriveDeadband = 0.1;
    public static final double kSlowDriveScaling = 0.25;
    public static final double kSuperSlowDriveScaling = .15;

    // Whether the driver wants to use the Logitech X3D flightstick controller.
    // If false, assume driver is using an Xbox controller.
    public static final boolean kUseX3D = true;
  }

  // Logitech flight controller button and joystick axes assignments.
  // Driver Controller: X3D Option
  final LogitechX3D driverX3DController = new LogitechX3D(Constants.kDriverX3DPort);
  final Trigger button1Trigger = new JoystickButton(driverX3DController, 1); // trigger
  final Trigger button2Trigger = new JoystickButton(driverX3DController, 2); // side thumb button
  final Trigger button3Trigger = new JoystickButton(driverX3DController, 3);
  final Trigger button5Trigger = new JoystickButton(driverX3DController, 5);
  final Trigger button7Trigger = new JoystickButton(driverX3DController, 7); // numbered buttons...
  final Trigger button8Trigger = new JoystickButton(driverX3DController, 8);
  final Trigger button9Trigger = new JoystickButton(driverX3DController, 9);
  final Trigger button10Trigger = new JoystickButton(driverX3DController, 10);
  final Trigger button11Trigger = new JoystickButton(driverX3DController, 11);
  final Trigger button12Trigger = new JoystickButton(driverX3DController, 12);

  final DoubleSupplier forwardAxisX3D = driverX3DController::getY;
  final DoubleSupplier sidewaysAxisX3D = driverX3DController::getX;
  final DoubleSupplier rotationAxisX3D = driverX3DController::getTwist;
  final DoubleSupplier sliderAxisX3D = driverX3DController::getThrottle;

  // // Driver controller : Xbox Option
  final XboxController driverXboxController = new XboxController(Constants.kDriverXboxPort);
  final Trigger startButtonDriver = new JoystickButton(driverXboxController, Button.kStart.value);
  final Trigger backButtonDriver = new JoystickButton(driverXboxController, Button.kStart.value);
  final Trigger leftBumperDriver = new JoystickButton(driverXboxController, Button.kLeftBumper.value);
  final Trigger rightBumperDriver = new JoystickButton(driverXboxController, Button.kRightBumper.value);
  final Trigger startAndBackButtonDriver = new Trigger(() -> {
    return startButtonDriver.getAsBoolean() && backButtonDriver.getAsBoolean();
  });
  final Trigger aButtonDriver = new JoystickButton(driverXboxController, Button.kA.value);
  final Trigger bButtonDriver = new JoystickButton(driverXboxController, Button.kB.value);
  final Trigger yButtonDriver = new JoystickButton(driverXboxController, Button.kY.value);
  final Trigger xButtonDriver = new JoystickButton(driverXboxController, Button.kX.value);
  final Trigger rightjoyAnalogTrigger = new Trigger(() -> {
    boolean triggerIsPressedEnough = driverXboxController.getRightTriggerAxis() > 0.28;
    return triggerIsPressedEnough;
  });
  final Trigger leftJoyAnalogTrigger = new Trigger(() -> {
    boolean triggerIsPressedEnough = driverXboxController.getLeftTriggerAxis() > 0.28;
    return triggerIsPressedEnough;
  });

  final DoubleSupplier forwardAxisXbox = driverXboxController::getLeftY;
  final DoubleSupplier sidewaysAxisXbox = driverXboxController::getLeftX;
  final DoubleSupplier rotationAxisXbox = driverXboxController::getRightX;

  // Operator controller (Xbox)
  final XboxController operatorController = new XboxController(Constants.kOperatorXboxPort);
  final Trigger leftBumperOperator = new JoystickButton(operatorController, Button.kLeftBumper.value);
  // final Trigger rightBumperOperator = new JoystickButton(operatorController,
  // Button.kRightBumper.value);
  final Trigger aButtonOperator = new JoystickButton(operatorController, Button.kA.value);
  final Trigger xButtonOperator = new JoystickButton(operatorController, Button.kX.value);
  final Trigger bButtonOperator = new JoystickButton(operatorController, Button.kB.value);
  final Trigger yButtonOperator = new JoystickButton(operatorController, Button.kY.value);
  final Trigger startButtonOperator = new JoystickButton(operatorController, Button.kStart.value);
  final Trigger backButtonOperator = new JoystickButton(operatorController, Button.kBack.value);
  final Trigger povUpOperator = new POVButton(operatorController, 0);
  final Trigger povDownOperator = new POVButton(operatorController, 180);
  final Trigger leftjoyTrigger = new JoystickButton(operatorController, Button.kLeftStick.value);
  final Trigger rightjoyTrigger = new JoystickButton(operatorController,
      Button.kRightStick.value);

  // Variables that will be set to either xbox or X3D axes in
  // configureButtonBindings().
  DoubleSupplier driverForwardAxis;
  DoubleSupplier driverSidewaysAxis;
  DoubleSupplier driverRotationAxis;

  Drivetrain drivetrain = new Drivetrain();
  Arm arm = new Arm();
  final Command recalibrateDrivetrain = new RunCommand(() -> drivetrain.recalibrateDrivetrain(), drivetrain);
  final Command moveArmToDefault = new RunCommand(() -> arm.move_Default(), arm);

  final Command moveArmToMid = new RunCommand(() -> arm.move_mid(), arm);

  final Command moveArmToAmp = new RunCommand(() -> arm.move_amp(), arm);

  final Command slowDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(driverForwardAxis.getAsDouble() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverSidewaysAxis.getAsDouble() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverRotationAxis.getAsDouble() * Constants.kSlowDriveScaling,
              Constants.kDriveDeadband),
          true, true),
      drivetrain);

  final Command fastDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(driverForwardAxis.getAsDouble(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverSidewaysAxis.getAsDouble(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverRotationAxis.getAsDouble(),
              Constants.kDriveDeadband),
          true, false),
      drivetrain);

  public RobotContainer() {
    configureButtonBindings();
    drivetrain.setDefaultCommand(fastDrive);
    arm.setDefaultCommand(moveArmToDefault);
  }

  public Command getAutonomousCommand() {
    var doNothing = new InstantCommand();
    return doNothing;
  }

  void configureButtonBindings() {

    if (Constants.kUseX3D) {
      driverForwardAxis = forwardAxisX3D;
      driverSidewaysAxis = sidewaysAxisX3D;
      driverRotationAxis = rotationAxisX3D;

      // Driver Joystick
      button2Trigger.whileTrue(slowDrive);
      button7Trigger.whileTrue(recalibrateDrivetrain);
      button11Trigger.whileTrue(moveArmToMid);
      button12Trigger.whileTrue(moveArmToAmp);
      button10Trigger.whileTrue(moveArmToDefault);

    } else {
      driverForwardAxis = forwardAxisXbox;
      driverSidewaysAxis = sidewaysAxisXbox;
      driverRotationAxis = rotationAxisXbox;

      // Driver Controller
      leftBumperDriver.whileTrue(slowDrive);
      startAndBackButtonDriver.whileTrue(recalibrateDrivetrain);

    }

    // Operator;

    // Sensors
  }

  void ensureSubsystemsHaveDefaultCommands() {
    drivetrain.setDefaultCommand(fastDrive);
    arm.setDefaultCommand(moveArmToDefault);
  }

}