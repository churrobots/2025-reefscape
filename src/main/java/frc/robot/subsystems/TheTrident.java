// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class TheTrident extends SubsystemBase {

  final TalonFX m_ReusedclimberFx = new TalonFX(Hardware.Trident.falconMotorCAN);

  final SparkMax m_tycoondropperSparkMax = new SparkMax(Hardware.Trident.neoMotorCAN, MotorType.kBrushless);
  final SparkClosedLoopController m_ = m_tycoondropperSparkMax.getClosedLoopController();

  final DigitalInput dropped = new DigitalInput(Hardware.Trident.dioID);

  public Command Dropthething() {
    return run(() -> {
      if (!dropped.get()) {
        m_tycoondropperSparkMax.set(0.1);
      } else {
        m_tycoondropperSparkMax.set(0);
      }
    });
  }

  public TheTrident() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
