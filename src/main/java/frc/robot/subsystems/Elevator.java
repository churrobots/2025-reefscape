// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class Elevator extends SubsystemBase {

  final TalonFX m_elevatorMotor = new TalonFX(Hardware.Elevator.leaderFalconMotorCAN);
  final TalonFX m_elevatorMotorFollow = new TalonFX(Hardware.Elevator.followerFalconMotorCAN);

  public Elevator() {
    setDefaultCommand(stop());
    // TODO: add PID values
  }

  public Command stop() {
    return run(() -> {
      m_elevatorMotor.stopMotor();
      m_elevatorMotorFollow.stopMotor();
    });
  }

  public Command move1Beta() {
    return run(() -> {
      double desiredOutputInRotations = 8;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      m_elevatorMotor.setPosition(requiredInputRotations);
    });
  }

  public Command move2Sigma() {
    return run(() -> {
      double desiredOutputInRotations = 16;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      m_elevatorMotor.setPosition(requiredInputRotations);
    });
  }

  public Command move3Alpha() {
    return run(() -> {
      double desiredOutputInRotations = 24;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      m_elevatorMotor.setPosition(requiredInputRotations);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
