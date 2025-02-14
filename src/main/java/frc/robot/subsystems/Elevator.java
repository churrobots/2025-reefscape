// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.HardwareRegistry;
import frc.robot.Hardware;

public class Elevator extends SubsystemBase {

  final TalonFX m_elevatorMotorLeader = new TalonFX(Hardware.Elevator.leaderFalconMotorCAN);
  final TalonFX m_elevatorMotorFollow = new TalonFX(Hardware.Elevator.followerFalconMotorCAN);
  final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0); // each move command will update the position of this
                                                                  // request and send it to the motor.

  class Constants {
    // PID numbers from:
    // https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/basic-pid-control.html
    static final double kP = 0.8; // An error of 1 rotation results in 2.4 V output
    static final double kI = 0.0; // no output for integrated error
    static final double kD = 0.0; // A velocity of 1 rps results in 0.1 V output
  }

  public Elevator() {
    setDefaultCommand(stop());
    HardwareRegistry.registerHardware(m_elevatorMotorLeader);
    HardwareRegistry.registerHardware(m_elevatorMotorFollow);

    talonFXConfigs.Slot0.kP = Constants.kP;
    talonFXConfigs.Slot0.kI = Constants.kI;
    talonFXConfigs.Slot0.kD = Constants.kD;

    // https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html
    // NOTE: these rps (rotations per second) values are on the motor side, before
    // the gearbox reduction. So the rotations of the sprocket will be fewer.
    talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 1; // Target cruise velocity of 1 rps
    talonFXConfigs.MotionMagic.MotionMagicAcceleration = 2; // Target acceleration of 2 rps/s (0.5 seconds to reach max
                                                            // target velocity of 1rps)
    talonFXConfigs.MotionMagic.MotionMagicJerk = 20; // Target jerk of 20 rps/s/s (0.1 seconds)

    // Brake when motor is not driven.
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_elevatorMotorLeader.getConfigurator().apply(talonFXConfigs);

    // Initialize encoder position to zero.
    m_elevatorMotorLeader.setPosition(0);

    m_elevatorMotorFollow.setControl(new Follower(m_elevatorMotorLeader.getDeviceID(), false));

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

      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

  // Move Elevator to L1 (trough).
  public Command move1Beta() {
    return run(() -> {
      double desiredVerticalTravelInMeters = 0.11;
      double desiredOutputInRotations = desiredVerticalTravelInMeters / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;

      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

  // Move Elevator to L2.
  public Command move2Sigma() {
    return run(() -> {
      double desiredVerticalTravelInMeters = 0.22;
      double desiredOutputInRotations = desiredVerticalTravelInMeters / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;

      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

  // Move Elevator to L3.
  public Command move3Alpha() {
    return run(() -> {
      double desiredVerticalTravelInMeters = 0.33;
      double desiredOutputInRotations = desiredVerticalTravelInMeters / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;

      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

}
