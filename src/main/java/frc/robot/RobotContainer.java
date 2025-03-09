// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pipeshooter;
import frc.robot.subsystems.UnnecessaryLEDS;

public class RobotContainer {
  Elevator elevator = new Elevator();
  Elbow elbow = new Elbow(elevator::getHeight);
  Pipeshooter pipeshooter = new Pipeshooter(elevator, elbow);
  Drivetrain drivetrain = new Drivetrain();
  UnnecessaryLEDS leds = new UnnecessaryLEDS();
  Climber climber = new Climber();

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
          () -> -1
              * MathUtil.applyDeadband(operatorXboxController.getLeftY(), xboxDeadband)
              * Hardware.DriverStation.slowbecauseyeah,
          () -> -1
              * MathUtil.applyDeadband(operatorXboxController.getLeftX(), xboxDeadband)
              * Hardware.DriverStation.slowbecauseyeah,
          () -> -1 * MathUtil.applyDeadband(operatorXboxController.getRightX(), xboxDeadband)
              * Hardware.DriverStation.slowbecauseyeah);

      drivetrain.setDefaultCommand(fastFieldRelativeDriverXboxControl);
      driverXboxController.rightBumper().whileTrue(slowFieldRelativeDriverXboxControl);
      driverXboxController.back().whileTrue(drivetrain.recalibrateDrivetrain());
      operatorXboxController.leftBumper()
          .whileTrue(slowRobotRelativeOperatorXboxControl.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
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
          .alongWith(pipeshooter.intakeCoral());
      operatorXboxController.a().whileTrue(moveElbowAndElevatorToRecieve);

      // TODO: add in ground algae command?
      // Command moveElbowAndElevatorToGroundAlgae =
      // elevator.moveToGroundAlgae().alongWith(elbow.aimToGroundAlgae())
      // .alongWith(pipeshooter.intakeCoral());
      // operatorXboxController.x().whileTrue(moveElbowAndElevatorToGroundAlgae);

      Command moveElbowAndElevatorTo1 = elevator.move1Beta().alongWith(elbow.aimAtTrough());
      operatorXboxController.x().onTrue(moveElbowAndElevatorTo1);

      Command moveElbowAndElevatorTo2 = elevator.move2Sigma().alongWith(elbow.aimAtReef());
      operatorXboxController.y().onTrue(moveElbowAndElevatorTo2);

      Command moveElbowAndElevatorTo3 = elevator.move3Alpha().alongWith(elbow.aimAtReef());
      operatorXboxController.b().onTrue(moveElbowAndElevatorTo3);

      operatorXboxController.back().whileTrue(elevator.recalibrateElevator());
      operatorXboxController.povUp().onTrue(elbow.aimAtAlgae().alongWith(elevator.moveToHighAlgae()));
      operatorXboxController.povDown().onTrue(elbow.aimAtAlgae().alongWith(elevator.moveToLowAlgae()));

      operatorXboxController.povRight().whileTrue(climber.moveUpwards());
      operatorXboxController.start().onTrue(climber.moveMid());
      operatorXboxController.povLeft().onTrue(climber.moveDown());
      // When we're not on a real field, make a command that we can use
      // for testing auto (putting the arm into position to hold our auto coral)
      if (!DriverStation.isFMSAttached()) {
        driverXboxController.povUp().onTrue(elbow.holdCoralHigh());
        driverXboxController.povDown().onTrue(elbow.aimToDump());
        driverXboxController.a().onTrue(pipeshooter.dumpCoral());
      }

      operatorXboxController.rightBumper()
          .whileTrue(pipeshooter.shootCoral().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
      operatorXboxController.rightStick().whileTrue(pipeshooter.intakeCoral());
    }

    Elastic.enableDashboardToBeDownloadedFromRobotDeployDirectory();
    SmartDashboard.putString("Robot Name", Hardware.robotName);

  }

  Supplier<Command> bindCommandsForAutonomous() {
    // FIXME: driver station is reporting that one of our autos uses a command that
    // doesn't exist in this map, check that and make sure to add it (or maybe it
    // was just a typo that needs to be fixed)
    NamedCommands.registerCommand("holdCoralHigh", elbow.holdCoralHigh());

    NamedCommands.registerCommand("aimToDump", elbow.aimToDump().withTimeout(3));
    NamedCommands.registerCommand("dumpCoral", pipeshooter.dumpCoral().withTimeout(2));

    NamedCommands.registerCommand("moveToHighAlgae",
        elbow.aimAtAlgae().alongWith(elevator.moveToHighAlgae().withTimeout(2)));
    NamedCommands.registerCommand("moveToLowAlgae",
        elbow.aimAtAlgae().alongWith(elevator.moveToLowAlgae().withTimeout(2)));

    NamedCommands.registerCommand("removeCoral",
        pipeshooter.shootCoral().withTimeout(2));
    NamedCommands.registerCommand("intakeCoral", elevator.moveToReceive().alongWith(elbow.receive())
        .alongWith(pipeshooter.intakeCoral()).withTimeout(3));
    NamedCommands.registerCommand("stopIntake", pipeshooter.idle());

    NamedCommands.registerCommand("waitForTeammates", new WaitCommand(9));

    SendableChooser<Command> autoChooser = drivetrain.createPathPlannerDropdown();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    return autoChooser::getSelected;
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