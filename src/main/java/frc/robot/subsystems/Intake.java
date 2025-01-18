// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.core.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANMapping;

public class Intake extends SubsystemBase {

  private final Pigeon2 fanumTaxIntaker = new Pigeon2(CANMapping.intakeMotor);

  public Intake() {
    fanumTaxIntaker.setInverted(true);
  }

  public boolean isYoinking() {
    boolean fanumTaxerIsIntaking = fanumTaxIntaker.get() > 0.5;
    if (fanumTaxerIsIntaking) {
      return true;
    } else {
      return false;
    }
  }

  public void yoinkTheRings() {
    fanumTaxIntaker.set(1);
  }

  public void deuceTheRings() {
    fanumTaxIntaker.set(-.35);
  }

  public boolean isDeucing() {
    boolean deucing = false;
    if (fanumTaxIntaker.get() < 0) {
      deucing = true;
    }
    return deucing;
  }

  public void ejectNow() {
    fanumTaxIntaker.set(-.85);
  }

  public void stopThePlan() {
    fanumTaxIntaker.set(0);
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