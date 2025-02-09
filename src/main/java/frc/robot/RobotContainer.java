// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.churrolib.vendor.Elastic;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pipeshooter;
import frc.robot.subsystems.UnnecessaryLEDS;

public class RobotContainer {

  Pipeshooter pipeshooter = new Pipeshooter();
  Elevator elevator = new Elevator();
  Elbow elbow = new Elbow();
  Drivetrain drivetrain = new Drivetrain();
  UnnecessaryLEDS leds = new UnnecessaryLEDS();

  void bindCommandsForTeleop() {

    CommandXboxController driverXboxController = new CommandXboxController(Hardware.DriverStation.driverXboxPort);
    CommandXboxController operatorXboxController = new CommandXboxController(Hardware.DriverStation.operatorXboxPort);

    Command recalibrateDriveTrain = new RunCommand(() -> drivetrain.recalibrateDrivetrain(), drivetrain);

    DoubleSupplier allianceRelativeFactor = () -> {
      boolean isBlueAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue;
      if (isBlueAlliance) {
        return 1.0;
      } else {
        return -1.0;
      }
    };
    double xboxDeadband = Hardware.DriverStation.driverXboxDeadband;

    Command fastFieldRelativeDriverXboxControl = drivetrain.createFieldRelativeDriveCommand(
        () -> -1 * allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(driverXboxController.getLeftY(), xboxDeadband)
            * Hardware.DriverStation.fastDriveScale,
        () -> -1 * allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(driverXboxController.getLeftX(), xboxDeadband)
            * Hardware.DriverStation.fastDriveScale,
        () -> -1 * MathUtil.applyDeadband(driverXboxController.getRightX(), xboxDeadband)
            * Hardware.DriverStation.fastDriveScale);

    Command slowFieldRelativeDriverXboxControl = drivetrain.createFieldRelativeDriveCommand(
        () -> -1 * allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(driverXboxController.getLeftY(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale,
        () -> -1 * allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(driverXboxController.getLeftX(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale,
        () -> -1 * MathUtil.applyDeadband(driverXboxController.getRightX(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale);

    Command slowRobotRelativeOperatorXboxControl = drivetrain.createRobotRelativeDriveCommand(
        () -> -1 * allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(operatorXboxController.getLeftY(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale,
        () -> -1 * allianceRelativeFactor.getAsDouble()
            * MathUtil.applyDeadband(operatorXboxController.getLeftX(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale,
        () -> -1 * MathUtil.applyDeadband(operatorXboxController.getRightX(), xboxDeadband)
            * Hardware.DriverStation.slowDriveScale);

    Command slowRobotRelativeOperatorXboxControlWithLEDs = slowRobotRelativeOperatorXboxControl.alongWith(
        leds.yuvrajalliseeisredwhenigoupsettyspaghetti());

    drivetrain.setDefaultCommand(fastFieldRelativeDriverXboxControl);
    driverXboxController.leftBumper().whileTrue(slowFieldRelativeDriverXboxControl);
    driverXboxController.back().whileTrue(recalibrateDriveTrain);

    operatorXboxController.leftBumper().whileTrue(slowRobotRelativeOperatorXboxControlWithLEDs);
    operatorXboxController.rightBumper().whileTrue(pipeshooter.shootCoral());

    // commands for the elbow positioning
    Command moveElbowAndElevatorToRecieve = elbow.recieve().alongWith(elevator.moveToRecieve())
        .alongWith(pipeshooter.intakeCoral()).alongWith(leds.jjisbeingasussybakaimpostoramongussus());
    operatorXboxController.a().whileTrue(moveElbowAndElevatorToRecieve);

    Command moveElbowAndElevatorTo1 = elbow.move1Beta().alongWith(elevator.move1Beta());
    operatorXboxController.x().onTrue(moveElbowAndElevatorTo1);

    Command moveElbowAndElevatorTo2 = elbow.move2Sigma().alongWith(elevator.move2Sigma().alongWith(leds.rainbow()));
    operatorXboxController.y().onTrue(moveElbowAndElevatorTo2);

    Command moveElbowAndElevatorTo3 = elbow.move2Sigma().alongWith(elevator.move3Alpha().alongWith(leds.blue()));
    operatorXboxController.b().onTrue(moveElbowAndElevatorTo3);

    Command moveElbowAndElevatorToL2Algae = elbow.moveAlgae().alongWith(elevator.move2Sigma().alongWith(leds.purple()));
    operatorXboxController.povDown().onTrue(moveElbowAndElevatorToL2Algae);

    Command moveElbowAndElevatorToL3Algae = elbow.moveAlgae().alongWith(elevator.move3Alpha().alongWith(leds.yellow()));
    operatorXboxController.povUp().onTrue(moveElbowAndElevatorToL3Algae);

    Elastic.enableDashboardToBeDownloadedFromRobotDeployDirectory();
    SmartDashboard.putString("Robot Name", Hardware.robotName);

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    NamedCommands.registerCommand("move1Beta", elbow.move1Beta());
    NamedCommands.registerCommand("move2Sigma", elbow.move2Sigma());
    NamedCommands.registerCommand("moveAlgae", elbow.moveAlgae());
    NamedCommands.registerCommand("intakePipeshooter", pipeshooter.intakeCoral());
    NamedCommands.registerCommand("shootCoral", pipeshooter.shootCoral());
    NamedCommands.registerCommand("waitForTeammates", new WaitCommand(9));

    // SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    // return autoChooser::getSelected;

    // TODO: fix this to grab the right path
    return drivetrain.getAutonomousCommand("Path IJ");
  }

  Supplier<Command> bindCommandsForAutonomous() {
    return () -> getAutonomousCommand();
  }

}
