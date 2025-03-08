// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.HardwareRegistry;
import frc.robot.Hardware;

public class Elevator extends SubsystemBase {

  final TalonFX m_elevatorMotorLeader = new TalonFX(Hardware.Elevator.leaderFalconMotorCAN);
  final TalonFX m_elevatorMotorFollow = new TalonFX(Hardware.Elevator.followerFalconMotorCAN);
  final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);
  double m_targetHeight = 0;

  public Elevator() {
    setDefaultCommand(moveToReceive());
    HardwareRegistry.registerHardware(m_elevatorMotorLeader);
    HardwareRegistry.registerHardware(m_elevatorMotorFollow);

    final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.Slot0.kS = Hardware.Elevator.kS;
    elevatorConfig.Slot0.kP = Hardware.Elevator.kP;
    elevatorConfig.Slot0.kI = Hardware.Elevator.kI;
    elevatorConfig.Slot0.kD = Hardware.Elevator.kD;
    elevatorConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static).kG = Hardware.Elevator.kG;

    // https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html
    // Motion Magic Voltage Control (Position):
    // 1) accelerates to cruise velocity
    // 2) maintains cruise velocity during cruise phase
    // 3) decelerates as it approaches the target position
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = Hardware.Elevator.kMaxVelocity;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = Hardware.Elevator.kMaxAcceleration;
    elevatorConfig.MotionMagic.MotionMagicJerk = Hardware.Elevator.kMaxJerk;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set the follower to follow the leader. The follower is not inverted due to
    // the geometry of the gearbox.
    // TODO: figure out why this doesn't work?
    // m_elevatorMotorFollow.setControl(new
    // Follower(m_elevatorMotorLeader.getDeviceID(), false));

    elevatorConfig.CurrentLimits.StatorCurrentLimit = Hardware.Elevator.kCurrentLimit;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = Hardware.Elevator.kCurrentLimit;
    m_elevatorMotorLeader.getConfigurator().apply(elevatorConfig);
    m_elevatorMotorFollow.getConfigurator().apply(elevatorConfig);

    setCurrentPositionAsZero();
  }

  // Move Elevator to Receiving position (this is the default position).
  public Command moveToReceive() {
    return moveToHeight(Hardware.Elevator.kReceiveHeight);
  }

  // Move Elevator to L1 (trough).
  public Command move1Beta() {
    return moveToHeight(Hardware.Elevator.kL1Height);
  }

  // Move Elevator to L2.
  public Command move2Sigma() {
    return moveToHeight(Hardware.Elevator.kL2Height);
  }

  // Move Elevator to L3.
  public Command move3Alpha() {
    return moveToHeight(Hardware.Elevator.kL3Height);
  }

  public Command moveToLowAlgae() {
    return moveToHeight(Hardware.Elevator.lowAlgaeHeighet);
  }

  public Command moveToGroundAlgae() {
    return moveToHeight(Hardware.Elevator.groundAlgaeHeight);
  }

  public Command moveToHighAlgae() {
    return moveToHeight(Hardware.Elevator.highAlgaeHeighet);
  }

  public Command recalibrateElevator() {
    return run(() -> {
      setCurrentPositionAsZero();
      m_elevatorMotorLeader.set(Hardware.Elevator.recalibrationSpeedPercentage);
      m_elevatorMotorFollow.set(Hardware.Elevator.recalibrationSpeedPercentage);
    });
  }

  private Command moveToHeight(double targetHeight) {
    return run(() -> {
      double safeHeight = MathUtil.clamp(
          targetHeight,
          Hardware.Elevator.minHeightInMeters,
          Hardware.Elevator.maxHeightInMeters);
      m_targetHeight = safeHeight;
      double desiredOutputInRotations = safeHeight / (Hardware.Elevator.sprocketPitchDiameter * Math.PI);
      double requiredInputRotations = Hardware.Elevator.gearboxReduction * desiredOutputInRotations;
      boolean shouldRestTheElevator = targetHeight < 0.02 && getHeight() < 0.01;
      if (shouldRestTheElevator) {
        m_elevatorMotorLeader.stopMotor();
        m_elevatorMotorFollow.stopMotor();
      } else {
        m_elevatorMotorLeader.setControl(m_positionRequest.withPosition(requiredInputRotations));
        m_elevatorMotorFollow.setControl(m_positionRequest.withPosition(requiredInputRotations));
      }
    });
  }

  private void setCurrentPositionAsZero() {
    m_elevatorMotorLeader.setPosition(0);
    m_elevatorMotorFollow.setPosition(0);
  }

  public double getHeight() {
    return m_elevatorMotorLeader.getPosition().getValueAsDouble() * Hardware.Elevator.sprocketPitchDiameter
        * Math.PI / Hardware.Elevator.gearboxReduction;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/CurrentHeight", getHeight());
    SmartDashboard.putNumber("Elevator/TargetHeight", m_targetHeight);
    SmartDashboard.putNumber("Elevator/OutputLeader", m_elevatorMotorLeader.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/OutputFollower", m_elevatorMotorFollow.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/StatorCurrentLeader",
        m_elevatorMotorLeader.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/StatorCurrentFollower",
        m_elevatorMotorFollow.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/SupplyCurrentLeader",
        m_elevatorMotorLeader.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/SupplyCurrentFollower",
        m_elevatorMotorFollow.getSupplyCurrent().getValueAsDouble());
  }

}
