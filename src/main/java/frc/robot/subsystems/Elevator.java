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
    static final double kS = 0.0; // static friction feed-forward component.
    static final double kV = 0.12; // motor-specific, represents the motor's velocity per volt. The reciprocal is
                                   // used as the feed-forward parameter in velocity-based control loops.
    static final double kA = 0.0; //
    static final double kP = 0.0; // proportional
    static final double kI = 0.0; // integral
    static final double kD = 0.0; // derivative

    // Maximum velocity in rotations per second of the post-gearbox sprocket (NOT of
    // the motor).
    static final double kMaxVelocity = 0.5 * Hardware.Elevator.gearboxReduction;
    static final double kMaxAcceleration = 1 * Hardware.Elevator.gearboxReduction;
    static final double kMaxJerk = 10 * Hardware.Elevator.gearboxReduction;

    // Desired Vertical Travel in meters for each position. Heights are specified
    // relative to the base position. Total elevator travel ranges from 0 to ~35cm.
    static final double kBaseHeight = 0.0; // initial height, also the height where we receive coral.
    static final double kL1Height = 0.11; // height to score in L1 trough
    static final double kL2Height = 0.22; // height to score in L2
    static final double kL3Height = 0.33; // height to score in L3
  }

  public Elevator() {
    setDefaultCommand(stop());
    HardwareRegistry.registerHardware(m_elevatorMotorLeader);
    HardwareRegistry.registerHardware(m_elevatorMotorFollow);

    talonFXConfigs.Slot0.kD = Constants.kS;
    talonFXConfigs.Slot0.kD = Constants.kV;
    talonFXConfigs.Slot0.kD = Constants.kA;
    talonFXConfigs.Slot0.kP = Constants.kP;
    talonFXConfigs.Slot0.kI = Constants.kI;
    talonFXConfigs.Slot0.kD = Constants.kD;

    // https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html
    // Motion Magic Voltage Control (Position):
    // 1) accelerates to cruise velocity
    // 2) maintains cruise velocity during cruise phase
    // 3) decelerates as it approaches the target position
    talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.kMaxVelocity;
    talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.kMaxAcceleration;
    talonFXConfigs.MotionMagic.MotionMagicJerk = Constants.kMaxJerk;

    // Brake when motor is not driven.
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply all above configs to the Leader.
    m_elevatorMotorLeader.getConfigurator().apply(talonFXConfigs);

    // Initialize encoder position to zero.
    m_elevatorMotorLeader.setPosition(0);

    // Set the follower to follow the leader. The follower is not inverted due to
    // the geometry of the gearbox.
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
      double desiredOutputInRotations = Constants.kBaseHeight / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;

      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

  // Move Elevator to L1 (trough).
  public Command move1Beta() {
    return run(() -> {
      double desiredOutputInRotations = Constants.kL1Height / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;

      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

  // Move Elevator to L2.
  public Command move2Sigma() {
    return run(() -> {
      double desiredOutputInRotations = Constants.kL2Height / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;

      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

  // Move Elevator to L3.
  public Command move3Alpha() {
    return run(() -> {
      double desiredOutputInRotations = Constants.kL3Height / Hardware.Elevator.sprocketPitchDiameter;
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;

      m_elevatorMotorLeader.setControl(m_request.withPosition(requiredInputRotations));
    });
  }

}
