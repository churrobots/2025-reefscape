// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.simulation.SimulationRegistry;
import frc.robot.Hardware;

public class Elevator extends SubsystemBase {

  final TalonFX m_elevatorMotorLeader = new TalonFX(Hardware.Elevator.leaderFalconMotorCAN);
  final TalonFX m_elevatorMotorFollow = new TalonFX(Hardware.Elevator.followerFalconMotorCAN);

  public Elevator() {
    setDefaultCommand(stop());
    SimulationRegistry.registerHardware(m_elevatorMotorLeader);
    SimulationRegistry.registerHardware(m_elevatorMotorFollow);
    // TODO: add PID configuration
  }

  public Command stop() {
    return run(() -> {
      m_elevatorMotorLeader.stopMotor();
      m_elevatorMotorFollow.stopMotor();
    });
  }

  public Command move1Beta() {
    return run(() -> {
      double desiredOutputInRotations = 8;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      m_elevatorMotorLeader.setPosition(requiredInputRotations);
    });
  }

  public Command move2Sigma() {
    return run(() -> {
      double desiredOutputInRotations = 16;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      m_elevatorMotorLeader.setPosition(requiredInputRotations);
    });
  }

  public Command move3Alpha() {
    return run(() -> {
      double desiredOutputInRotations = 24;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      m_elevatorMotorLeader.setPosition(requiredInputRotations);
    });
  }

}
