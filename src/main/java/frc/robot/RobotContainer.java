// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.churrolib.vendor.Elastic;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.OperatorCamera;
import frc.robot.subsystems.Pipeshooter;

public class RobotContainer {

  Pipeshooter pipeshooter = new Pipeshooter();
  Elevator elevator = new Elevator();
  Elbow elbow = new Elbow();
  Drivetrain drivetrain = new Drivetrain();
  OperatorCamera operatorCamera = new OperatorCamera();

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

    drivetrain.setDefaultCommand(fastFieldRelativeDriverXboxControl);
    driverXboxController.leftBumper().whileTrue(slowFieldRelativeDriverXboxControl);
    driverXboxController.back().whileTrue(recalibrateDriveTrain);

    operatorXboxController.leftBumper().whileTrue(slowRobotRelativeOperatorXboxControl);

    if (pipeshooter != null) {
      operatorXboxController.rightBumper().whileTrue(pipeshooter.shootCoral());
    }

    // commands for the elbow positioning
    Command moveElbowAndElevatorToRecieve = elbow.recieve().alongWith(elevator.moveToRecieve())
        .alongWith(pipeshooter.intakeCoral());
    operatorXboxController.a().whileTrue(moveElbowAndElevatorToRecieve);

    Command moveElbowAndElevatorTo1 = elbow.move1Beta().alongWith(elevator.move1Beta());
    operatorXboxController.x().onTrue(moveElbowAndElevatorTo1);

    Command moveElbowAndElevatorTo2 = elbow.move2Sigma().alongWith(elevator.move2Sigma());
    operatorXboxController.y().onTrue(moveElbowAndElevatorTo2);

    Command moveElbowAndElevatorTo3 = elbow.move2Sigma().alongWith(elevator.move3Alpha());
    operatorXboxController.b().onTrue(moveElbowAndElevatorTo3);

    Command moveElbowAndElevatorToL2Algae = elbow.moveAlgae().alongWith(elevator.move2Sigma());
    operatorXboxController.povDown().onTrue(moveElbowAndElevatorToL2Algae);

    Command moveElbowAndElevatorToL3Algae = elbow.moveAlgae().alongWith(elevator.move3Alpha());
    operatorXboxController.povUp().onTrue(moveElbowAndElevatorToL3Algae);

    if (Hardware.DriverStation.useLowQualityCamera) {
      operatorCamera.startLowQualityStream();
    } else {
      operatorCamera.startHighQualityStream();
    }

    Elastic.enableDashboardToBeDownloadedFromRobotDeployDirectory();
    SmartDashboard.putString("Robot Name", Hardware.robotName);

  }

  Supplier<Command> bindCommandsForAutonomous() {

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          drivetrain::getPose, // Robot pose supplier
          drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> drivetrain.setRobotRelativeSpeeds(speeds), // Method that will drive the robot given
                                                                               // ROBOT RELATIVE ChassisSpeeds. Also
                                                                               // optionally outputs individual module
                                                                               // feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            boolean isBlueAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue;
            boolean shouldFlip = !isBlueAlliance;
            return shouldFlip;
          },
          drivetrain // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // TODO: make real commands for auto to use
    NamedCommands.registerCommand("move1Beta", elbow.move1Beta());
    NamedCommands.registerCommand("move2Sigma", elbow.move2Sigma());
    NamedCommands.registerCommand("moveAlgae", elbow.moveAlgae());
    NamedCommands.registerCommand("intakePipeshooter", pipeshooter.intakeCoral());
    NamedCommands.registerCommand("shootCoral", pipeshooter.shootCoral());
    NamedCommands.registerCommand("waitForTeammates", new WaitCommand(9));

    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    return autoChooser::getSelected;

  }

}
