// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class falconshooter extends SubsystemBase {
  /** Creates a new falconshooter. */
  public falconshooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // The Falcon motor
  private TalonFX falconMotor;

  @Override
  public void() {
      // Initialize the motor on CAN ID 1
      falconMotor = new TalonFX(1);
  }

  @Override
  public void() {
      // Set the motor to spin at 30% power when teleop starts
      falconMotor.setControl(new DutyCycleOut(0.3));
  }

  @Override
  public void() {
      // Nothing else needed – motor keeps running
  }

  public TalonFX getFalconMotor() {
    return falconMotor;
  }

  public void setFalconMotor(TalonFX falconMotor) {
    this.falconMotor = falconMotor;
  }
}