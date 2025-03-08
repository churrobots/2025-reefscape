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

public class Climber extends SubsystemBase {
  final TalonFX m_climberMotor = new TalonFX(Hardware.Climber.falconMotorCAN);
  final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);
  double m_targetRotations = 0;

  /** Creates a new Climber. */
  public Climber() {
    setDefaultCommand(moveDown());
    HardwareRegistry.registerHardware(m_climberMotor);
    
    final TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.Slot0.kS = Hardware.Climber.kS;
    climberConfig.Slot0.kP = Hardware.Climber.kP;
    climberConfig.Slot0.kI = Hardware.Climber.kI;
    climberConfig.Slot0.kD = Hardware.Climber.kD;
    climberConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine).kG = Hardware.Climber.kG;

    // https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html
    // Motion Magic Voltage Control (Position):
    // 1) accelerates to cruise velocity
    // 2) maintains cruise velocity during cruise phase
    // 3) decelerates as it approaches the target position
    climberConfig.MotionMagic.MotionMagicCruiseVelocity = Hardware.Climber.kMaxVelocity;
    climberConfig.MotionMagic.MotionMagicAcceleration = Hardware.Climber.kMaxAcceleration;
    climberConfig.MotionMagic.MotionMagicJerk = Hardware.Climber.kMaxJerk;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climberConfig.CurrentLimits.StatorCurrentLimit = Hardware.Climber.kCurrentLimit;
    climberConfig.CurrentLimits.SupplyCurrentLimit = Hardware.Climber.kCurrentLimit;
    m_climberMotor.getConfigurator().apply(climberConfig);

    setCurrentPositionAsZero();

  }

  private void setCurrentPositionAsZero() {
    m_climberMotor.setPosition(0);
  }

  public Command moveDown() {
    return moveToRotations(Hardware.Climber.kDown);
  }

  public Command moveMid() {
    return moveToRotations(Hardware.Climber.kMid);
  }

  public Command moveUp() {
    return moveToRotations(Hardware.Climber.kUp);
  }

  public double getRotations() {
    return m_climberMotor.getPosition().getValueAsDouble() / Hardware.Climber.gearboxReduction;
  }

  private Command moveToRotations(double desiredOutputInRotations) {
    return run(() -> {
      double safeRotations = MathUtil.clamp(
          desiredOutputInRotations,
          Hardware.Climber.minRotations,
          Hardware.Climber.maxRotations);
      m_targetRotations = safeRotations;
      double requiredInputRotations = Hardware.Climber.gearboxReduction * desiredOutputInRotations;
      m_climberMotor.setControl(m_positionRequest.withPosition(requiredInputRotations));
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber/CurrentRotations", getRotations());
    SmartDashboard.putNumber("Climber/TargetRotations", m_targetRotations);
    SmartDashboard.putNumber("Climber/Output", m_climberMotor.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Climber/StatorCurrent",
        m_climberMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climber/SupplyCurrent",
        m_climberMotor.getSupplyCurrent().getValueAsDouble());
  }
}
