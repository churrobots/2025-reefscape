// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class Intake extends SubsystemBase {

  private final VictorSPX fanumTaxIntaker = new VictorSPX(Hardware.Intake.brushedMotorCAN);

  public Intake() {
    setDefaultCommand(new RunCommand(this::stopThePlan, this));
    fanumTaxIntaker.setInverted(true);
  }

  public void yoink() {
    fanumTaxIntaker.set(ControlMode.PercentOutput, 1);
  }

  public void stopThePlan() {
    fanumTaxIntaker.set(ControlMode.PercentOutput, 0);
  }

  public void periodic() {
    SmartDashboard.putNumber("Intake Motor Output", fanumTaxIntaker.getMotorOutputPercent());
  }

}

// THE WORLD LOVES ETHAN
// YES KING
// BETZY IS A HATER
// DANIEL IS OFF THE MEDS
// NU UH
// MATEOS AN OPP
// BETZY'S A SNITCH
// DANIEL IS SUS
// TATI IS THE BEST CAPPPPPP SHE'S THE WORST ON HOOD
// DANIEL IS BANNED FROM COMING
// MAY 24TH THE NEW KINGS DANIEL 2.0 (THE SEQUEL) AND ALDO THE GOAT ARRIVED
// THE GOAT DANIELSAN MADE AN AUTO AT CHEZY
// THE SUPER SIGMA DAINEL SAN MADE CODE FOR JJ'S STICK
// THE GOAT