// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.churrolib.CommandXboxControllerForSimulation;
import frc.churrolib.HardwareRegistry;
import frc.churrolib.vendor.Elastic;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pipeshooter;
import frc.robot.subsystems.UnnecessaryLEDS;

public class RobotContainer {

  Pipeshooter pipeshooter = new Pipeshooter();
  Elevator elevator = new Elevator();
  Elbow elbow = new Elbow(elevator::getHeight);
  Drivetrain drivetrain = new Drivetrain();
  UnnecessaryLEDS leds = new UnnecessaryLEDS();

  void bindCommandsForTeleop() {

    CommandXboxController driverXboxController = RobotBase.isSimulation()
        && DriverStation.isJoystickConnected(Hardware.DriverStation.driverSimulationXboxPort)
            ? new CommandXboxControllerForSimulation(Hardware.DriverStation.driverSimulationXboxPort)
            : new CommandXboxController(Hardware.DriverStation.driverXboxPort);

    CommandXboxController operatorXboxController = new CommandXboxController(Hardware.DriverStation.operatorXboxPort);

    DoubleSupplier allianceRelativeFactor = () -> {
      boolean isBlueAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue;
      if (isBlueAlliance) {
        return 1.0;
      } else {
        return -1.0;
      }
    };
    double xboxDeadband = Hardware.DriverStation.driverXboxDeadband;

    // TODO: don't make drivetrain conditional, this is something wrong with a
    // silent exception, maybe when sparkmaxes are not connected?
    // TODO: also if it IS the sparkmaxes being disconnected, that's bad, because
    // the robot shouldn't crash entirely if we lose one motor
    if (drivetrain != null) {
      Command recalibrateDriveTrain = new RunCommand(() -> drivetrain.recalibrateDrivetrain(), drivetrain);
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

      drivetrain.setDefaultCommand(fastFieldRelativeDriverXboxControl);
      driverXboxController.rightBumper().whileTrue(slowFieldRelativeDriverXboxControl);
      driverXboxController.back().whileTrue(recalibrateDriveTrain);
      operatorXboxController.leftBumper().whileTrue(slowRobotRelativeOperatorXboxControl.alongWith(leds.red()));
    }

    if (Hardware.DriverStation.mechanismsAreInTestMode) {
      // This is for safely testing the beta bot in the meantime
      // operatorXboxController.x().whileTrue(elevator.move1Beta());
      operatorXboxController.y().whileTrue(elevator.move2Sigma());
      operatorXboxController.b().whileTrue(elevator.move3Alpha());
      operatorXboxController.rightBumper().whileTrue(pipeshooter.shootCoral());
      operatorXboxController.povDown().whileTrue(pipeshooter.intakeCoral());

      operatorXboxController.a().whileTrue(elbow.receive());
      operatorXboxController.leftBumper().whileTrue(elbow.aimAtReef());

      operatorXboxController.x().whileTrue(
          elevator.move3Alpha().alongWith(elbow.aimAtReef()));
      // operatorXboxController.rightBumper().whileTrue(elbow.aimAtAlgae());

    } else {
      Command moveElbowAndElevatorToRecieve = elevator.moveToReceive().alongWith(elbow.receive())
          .alongWith(pipeshooter.intakeCoral()).alongWith(leds.jjisbeingasussybakaimpostoramongussus());
      operatorXboxController.a().whileTrue(moveElbowAndElevatorToRecieve);

      Command moveElbowAndElevatorTo1 = elevator.move1Beta().alongWith(elbow.aimAtReef());
      operatorXboxController.x().onTrue(moveElbowAndElevatorTo1);

      Command moveElbowAndElevatorTo2 = elevator.move2Sigma().alongWith(elbow.aimAtReef().alongWith(leds.rainbow()));
      operatorXboxController.y().onTrue(moveElbowAndElevatorTo2);

      Command moveElbowAndElevatorTo3 = elbow.aimAtReef().alongWith(leds.blue());
      operatorXboxController.b().onTrue(moveElbowAndElevatorTo3);

      Command moveElbowAndElevatorToL2Algae = elbow.aimAtAlgae()
          .alongWith(elevator.move2Sigma().alongWith(leds.purple()));
      operatorXboxController.povDown().onTrue(moveElbowAndElevatorToL2Algae);

      Command moveElbowAndElevatorToL3Algae = elbow.aimAtAlgae()
          .alongWith(elevator.move3Alpha().alongWith(leds.yellow()));
      operatorXboxController.povUp().onTrue(moveElbowAndElevatorToL3Algae);

      Command shootCoral = pipeshooter.shootCoral().alongWith(leds.green());
      operatorXboxController.rightBumper().whileTrue(shootCoral);
    }

    Elastic.enableDashboardToBeDownloadedFromRobotDeployDirectory();
    SmartDashboard.putString("Robot Name", Hardware.robotName);

  }

  Command showCommand(String text) {
    return new InstantCommand(() -> {
      SmartDashboard.putString("AutoCommand", text);
    });
  }

  Supplier<Command> bindCommandsForAutonomous() {
    NamedCommands.registerCommand("Intake Coral", pipeshooter.intakeCoral());

    NamedCommands.registerCommand(
        "shootL1",
        elbow.aimAtReef()
            .alongWith(elevator.move1Beta())
            .andThen(pipeshooter.shootCoral())
            .andThen(elbow.receive().alongWith(elevator.moveToReceive()))
            .alongWith(showCommand("Shoot L1"))
            .alongWith(leds.green()));

    NamedCommands.registerCommand(
        "shootL2",
        elbow.aimAtReef()
            .alongWith(elevator.move2Sigma())
            .andThen(pipeshooter.shootCoral())
            .andThen(elbow.receive().alongWith(elevator.moveToReceive()))
            .alongWith(showCommand("Shoot L2"))
            .alongWith(leds.green()));

    NamedCommands.registerCommand("moveAlgae", elbow.aimAtAlgae().alongWith(showCommand("Move Algae")));
    NamedCommands.registerCommand("intakePipeshooter",
        pipeshooter.intakeCoral().withTimeout(2).alongWith(showCommand("intakePipeshooter")));
    NamedCommands.registerCommand("shootCoral",
        pipeshooter.shootCoral().alongWith(showCommand("shootCoral")).withTimeout(2));
    NamedCommands.registerCommand("waitForTeammates", new WaitCommand(9).alongWith(showCommand("wait For Teammates")));

    // TODO: we shouldn't have the drivetrain null but we need to since there is a
    // silent crasher on beta bot
    if (drivetrain != null) {
      SendableChooser<Command> autoChooser = drivetrain.createPathPlannerDropdown();
      SmartDashboard.putData("Auto Chooser", autoChooser);
      return autoChooser::getSelected;
    } else {
      return () -> Commands.none();
    }
  }

  void updateDiagnostics() {
    HardwareRegistry.dumpDeviceFaultsToNetworkTables();
    if (Hardware.Diagnostics.debugMemoryLeaks) {
      long allocatedMemoryInBytes = (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory());
      long presumableFreeMemoryInBytes = Runtime.getRuntime().maxMemory() - allocatedMemoryInBytes;
      SmartDashboard.putNumber("freeMemory", presumableFreeMemoryInBytes);
    }
  }

}