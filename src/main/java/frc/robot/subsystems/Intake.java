// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.core.*;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANMapping;

public class Intake extends SubsystemBase {

  private final VictorSPX fanumTaxIntaker = new VictorSPX(CANMapping.intakeMotor);

  public Intake() {
    fanumTaxIntaker.setInverted(true);
  }

  public boolean isYoinking() {
    boolean fanumTaxerIsIntaking = fanumTaxIntaker.getMotorOutputPercent() > 0.5;
    if (fanumTaxerIsIntaking) {
      return true;
    } else {
      return false;
    }
  }

  public void yoinkTheRings() {
    fanumTaxIntaker.set(ControlMode.PercentOutput, 1);
  }

  public void deuceTheRings() {
    fanumTaxIntaker.set(ControlMode.PercentOutput, -.35);
  }

  public boolean isDeucing() {
    boolean deucing = false;
    if (fanumTaxIntaker.getMotorOutputPercent() < 0) {
      deucing = true;
    }
    return deucing;
  }

  public void ejectNow() {
    fanumTaxIntaker.set(ControlMode.PercentOutput, -.85);
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