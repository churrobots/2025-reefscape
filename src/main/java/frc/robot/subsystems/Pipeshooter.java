// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.simulation.SimulationRegistry;
import frc.robot.Hardware;

// TODO(Controls): (1) Clarify the below understanding of the general robot design. Understand what sensors will be part of each subsystem.
// Elevator: motor(s) with gearbox to drive elevator up and down
// Arm: motor with gearbox to pivot arm
// Pipeshooter: motor(s), potentially with gearbox, to drive intake and outtake (aka shooting)
//
// (2) given the design, skeleton the required files for the subsystems.
// -- for example, may need an PivotArm.java and Elevator.java, can probably delete Intake.java.
//
// (3) Create specific commands for the subsystems, e.g. Elevator needs to be able to go to various positions
//
// (4) Create button mappings for the commands from (3).

public class Pipeshooter extends SubsystemBase {

  final TalonFX m_pipeShooterMotor = new TalonFX(Hardware.Shooter.falconMotorCAN);

  public Pipeshooter() {
    setDefaultCommand(stop());
    SimulationRegistry.registerHardware(m_pipeShooterMotor);
    // TODO: add PID configuration
  }

  public Command stop() {
    return run(() -> {
      m_pipeShooterMotor.set(0);
    });
  }

  public Command intakeCoral() {
    return run(() -> {
      m_pipeShooterMotor.set(1);

    });
  }

  public Command shootCoral() {
    return run(() -> {
      m_pipeShooterMotor.set(-1);
    });
  }

}
