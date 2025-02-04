// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.simulation.SimulationRegistry;
import frc.robot.Hardware;

public class Elevator extends SubsystemBase {

  final TalonFX m_elevatorMotorLeader = new TalonFX(Hardware.Elevator.leaderFalconMotorCAN);
  final TalonFX m_elevatorMotorFollow = new TalonFX(Hardware.Elevator.followerFalconMotorCAN);

  class Constants {
    // PID numbers from:
    // https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/basic-pid-control.html
    static final double kP = 2.4; // An error of 1 rotation results in 2.4 V output
    static final double kI = 0.0; // no output for integrated error
    static final double kD = 0.0; // A velocity of 1 rps results in 0.1 V output
  }

  public Elevator() {
    setDefaultCommand(stop());
    SimulationRegistry.registerHardware(m_elevatorMotorLeader);
    SimulationRegistry.registerHardware(m_elevatorMotorFollow);
    // TODO: add PID configuration

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.kP;
    slot0Configs.kI = Constants.kI;
    slot0Configs.kD = Constants.kD;

    m_elevatorMotorLeader.getConfigurator().apply(slot0Configs);

  }

  public Command stop() {
    return run(() -> {
      m_elevatorMotorLeader.stopMotor();
      m_elevatorMotorFollow.stopMotor();
    });
  }

  // Intake
  public Command moveToRecieve() {
    return run(() -> {
      double desiredOutputInRotations = 4;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      // create a position closed-loop request, voltage output, slot 0 configs
      final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

      // set position to 10 rotations
      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

  // trough
  public Command move1Beta() {
    return run(() -> {
      double desiredOutputInRotations = 8;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      // create a position closed-loop request, voltage output, slot 0 configs
      final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

      // set position to 10 rotations
      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

  // L2
  public Command move2Sigma() {
    return run(() -> {
      double desiredOutputInRotations = 16;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      // create a position closed-loop request, voltage output, slot 0 configs
      final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

      // set position to 10 rotations
      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

  // L3
  public Command move3Alpha() {
    return run(() -> {
      double desiredOutputInRotations = 24;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      // create a position closed-loop request, voltage output, slot 0 configs
      final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

      // set position to 10 rotations
      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

}
