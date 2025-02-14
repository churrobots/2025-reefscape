// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.HardwareRegistry;
import frc.robot.Hardware;

public class Elevator extends SubsystemBase {

  final TalonFX m_elevatorMotorLeader = new TalonFX(Hardware.Elevator.leaderFalconMotorCAN);
  final TalonFX m_elevatorMotorFollow = new TalonFX(Hardware.Elevator.followerFalconMotorCAN);
  final Slot0Configs slot0Configs = new Slot0Configs();

  class Constants {
    // PID numbers from:
    // https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/basic-pid-control.html
    static final double kP = 2.4; // An error of 1 rotation results in 2.4 V output
    static final double kI = 0.0; // no output for integrated error
    static final double kD = 0.0; // A velocity of 1 rps results in 0.1 V output
  }

  public Elevator() {
    setDefaultCommand(stop());
    HardwareRegistry.registerHardware(m_elevatorMotorLeader);
    HardwareRegistry.registerHardware(m_elevatorMotorFollow);

    slot0Configs.kP = Constants.kP;
    slot0Configs.kI = Constants.kI;
    slot0Configs.kD = Constants.kD;

    m_elevatorMotorLeader.getConfigurator().apply(slot0Configs);
    m_elevatorMotorLeader.setPosition(0); // Initializing the encoder position to 0
  }

  public Command stop() {
    return run(() -> {
      m_elevatorMotorLeader.stopMotor();
      m_elevatorMotorFollow.stopMotor();
    });
  }

  // Move Elevator to Receiving position (this is the default position).
  public Command moveToReceive() {
    return run(() -> {
      double desiredVerticalTravelInMeters = 0;
      double desiredOutputInRotations = desiredVerticalTravelInMeters / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      // create a position closed-loop request, voltage output, slot 0 configs
      final PositionVoltage m_request = new PositionVoltage(requiredInputRotations).withSlot(0);

      m_elevatorMotorLeader.setControl(m_request);
    });
  }

  // Move Elevator to L1 (trough).
  public Command move1Beta() {
    return run(() -> {
      double desiredVerticalTravelInMeters = 0.115;
      double desiredOutputInRotations = desiredVerticalTravelInMeters / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      // create a position closed-loop request, voltage output, slot 0 configs
      final PositionVoltage m_request = new PositionVoltage(requiredInputRotations).withSlot(0);

      m_elevatorMotorLeader.setControl(m_request);
    });
  }

  // Move Elevator to L2.
  public Command move2Sigma() {
    return run(() -> {
      double desiredVerticalTravelInMeters = 0.22;
      double desiredOutputInRotations = desiredVerticalTravelInMeters / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      // create a position closed-loop request, voltage output, slot 0 configs
      final PositionVoltage m_request = new PositionVoltage(requiredInputRotations).withSlot(0);

      m_elevatorMotorLeader.setControl(m_request);
    });
  }

  // Move Elevator to L3.
  public Command move3Alpha() {
    return run(() -> {
      double desiredVerticalTravelInMeters = 0.33;
      double desiredOutputInRotations = desiredVerticalTravelInMeters / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      // create a position closed-loop request, voltage output, slot 0 configs
      final PositionVoltage m_request = new PositionVoltage(requiredInputRotations).withSlot(0);

      m_elevatorMotorLeader.setControl(m_request);
    });
  }

}
