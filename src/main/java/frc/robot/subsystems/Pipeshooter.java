// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.HardwareRegistry;
import frc.robot.Hardware;

public class Pipeshooter extends SubsystemBase {

  final TalonFX m_pipeShooterMotor = new TalonFX(Hardware.Pipeshooter.falconMotorCAN);
  final Elevator m_elevator;
  final Elbow m_elbow;

  public Pipeshooter(Elevator elevator, Elbow elbow) {
    m_elevator = elevator;
    m_elbow = elbow;
    final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_pipeShooterMotor.getConfigurator().apply(shooterConfig);
    HardwareRegistry.registerHardware(m_pipeShooterMotor);
    setDefaultCommand(idle());
  }

  public Command idle() {
    return run(() -> {
      if (DriverStation.isAutonomous()) {
        m_pipeShooterMotor.set(-0.05);
      } else {
        m_pipeShooterMotor.set(-0.05);
      }

    });
  }

  public Command intakeCoral() {
    return run(() -> {
      m_pipeShooterMotor.set(-0.40);
    });
  }

  public Command dumpCoral() {
    return run(() -> {
      m_pipeShooterMotor.set(-0.050);
    });
  }

  public Command shootCoral() {
    return run(() -> {
      boolean notSafe = m_elbow.getCurrentElbowPosition() < Hardware.Elbow.minimumRotationsForSafeShooting;
      if (notSafe) {
        return;
      } else if (m_elevator.getHeight() > Hardware.Elevator.kL1Height) {
        m_pipeShooterMotor.set(0.40);
      } else {
        m_pipeShooterMotor.set(0.20);
      }
    });

  }

}
