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
  Elbow elbow = new Elbow();
  Drivetrain drivetrain = Hardware.Drivetrain.disableDrivetrainDueToSilentBootFailure ? null : new Drivetrain();
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

      Command slowRobotRelativeOperatorXboxControlWithLEDs = slowRobotRelativeOperatorXboxControl.alongWith(
          leds.yuvrajalliseeisredwhenigoupsettyspaghetti());

      drivetrain.setDefaultCommand(fastFieldRelativeDriverXboxControl);
      driverXboxController.leftBumper().whileTrue(slowFieldRelativeDriverXboxControl);
      driverXboxController.back().whileTrue(recalibrateDriveTrain);
      operatorXboxController.leftBumper().whileTrue(slowRobotRelativeOperatorXboxControlWithLEDs);
    }

    if (Hardware.DriverStation.mechanismsAreInTestMode) {
      // This is for safely testing the beta bot in the meantime
      operatorXboxController.x().whileTrue(elevator.move1Beta());
      operatorXboxController.y().whileTrue(elevator.move2Sigma());
      operatorXboxController.b().whileTrue(elevator.move3Alpha());
      operatorXboxController.a().whileTrue(pipeshooter.intakeCoral());
      operatorXboxController.rightBumper().whileTrue(pipeshooter.shootCoral());

    } else {
      Command moveElbowAndElevatorToRecieve = elbow.recieve().alongWith(elevator.moveToReceive())
          .alongWith(pipeshooter.intakeCoral()).alongWith(leds.jjisbeingasussybakaimpostoramongussus());
      operatorXboxController.a().whileTrue(moveElbowAndElevatorToRecieve);

      Command moveElbowAndElevatorTo1 = elbow.move1Beta().alongWith(elevator.move1Beta());
      operatorXboxController.x().onTrue(moveElbowAndElevatorTo1);

      Command moveElbowAndElevatorTo2 = elbow.move2Sigma().alongWith(elevator.move2Sigma().alongWith(leds.rainbow()));
      operatorXboxController.y().onTrue(moveElbowAndElevatorTo2);

      Command moveElbowAndElevatorTo3 = elbow.move2Sigma().alongWith(elevator.move3Alpha().alongWith(leds.blue()));
      operatorXboxController.b().onTrue(moveElbowAndElevatorTo3);

      Command moveElbowAndElevatorToL2Algae = elbow.moveAlgae()
          .alongWith(elevator.move2Sigma().alongWith(leds.purple()));
      operatorXboxController.povDown().onTrue(moveElbowAndElevatorToL2Algae);

      Command moveElbowAndElevatorToL3Algae = elbow.moveAlgae()
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
        elbow.move1Beta()
            .alongWith(elevator.move1Beta())
            .andThen(pipeshooter.shootCoral())
            .andThen(elbow.recieve().alongWith(elevator.moveToReceive()))
            .alongWith(showCommand("Shoot L1"))
            .alongWith(leds.green()));

    NamedCommands.registerCommand(
        "shootL2",
        elbow.move1Beta()
            .alongWith(elevator.move2Sigma())
            .andThen(pipeshooter.shootCoral())
            .andThen(elbow.recieve().alongWith(elevator.moveToReceive()))
            .alongWith(showCommand("Shoot L2"))
            .alongWith(leds.green()));

    NamedCommands.registerCommand("moveAlgae", elbow.moveAlgae().alongWith(showCommand("Move Algae")));
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
  }

}