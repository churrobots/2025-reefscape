// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.HardwareRegistry;
import frc.robot.Hardware;

public class Pipeshooter extends SubsystemBase {

  final TalonFX m_pipeShooterMotor = new TalonFX(Hardware.Pipeshooter.falconMotorCAN);

  public Pipeshooter() {
    setDefaultCommand(stop());
    HardwareRegistry.registerHardware(m_pipeShooterMotor);
  }

  public Command stop() {
    return run(() -> {
      m_pipeShooterMotor.set(0);
    });
  }

  public Command intakeCoral() {
    return run(() -> {
      m_pipeShooterMotor.set(-0.15);
    });
  }

  public Command shootCoral() {
    return run(() -> {
      m_pipeShooterMotor.set(0.15);
    }).withTimeout(1);
  }

}
