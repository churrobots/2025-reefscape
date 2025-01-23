// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib.simulation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulates a roller system driven by a Falcon.
 * 
 * Inspiration taken from Quixilver, with the arm removed. Reference back
 * here if we want to figure out how to properly attach a roller to an arm.
 * https://github.com/frc604/robot-sim-example/blob/main/src/main/java/frc/robot/subsystems/IntakeSubsystem.java
 */
public class CTRESingleFalconRollerSim {

  final TalonFX m_rollerMotor;
  final TalonFXSimState m_rollerMotorSim;
  final FlywheelSim m_rollerPhysicsSim;

  public CTRESingleFalconRollerSim(
      TalonFX rollerMotor,
      double rollerReduction,
      double rollerMOI) {

    m_rollerMotor = rollerMotor;
    m_rollerMotorSim = m_rollerMotor.getSimState();

    // Rollers are basically flywheels.
    m_rollerPhysicsSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), rollerMOI, rollerReduction),
        DCMotor.getFalcon500(1));
  }

  public double rollerOutputVelocityRPM() {
    return m_rollerPhysicsSim.getAngularVelocityRPM();
  }

  public void iterate(double timeDeltaInSeconds) {
    // Apply the motor controller's voltage to the roller wheel physics sim for
    // the number of seconds given.
    m_rollerPhysicsSim.setInput(m_rollerMotorSim.getMotorVoltage());
    m_rollerPhysicsSim.update(timeDeltaInSeconds);

    // Then we can ask the driving physics sim what the resulting velocity is, so we
    // can apply that velocity to the driving motor controller sim, so that the
    // motor controller can update its simulated sensors for distance and velocity.
    // Note that the physical output goes through the gear reduction before reaching
    // the relative encoder on the motor itself, hence the `drivingEncoderRPM` step.
    // Ultimately, the `iterate()` method takes converted units.
    // NOTE: Make sure to convert radians at the mechanism to rotations at the motor
    // Also, subtract out the starting angle is necessary so the simulation can't
    // "cheat" and use the sim as an absolute encoder.
    double rotationsPerSecond = m_rollerPhysicsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    m_rollerMotorSim.setRotorVelocity(rotationsPerSecond);
    m_rollerMotorSim.addRotorPosition(rotationsPerSecond * timeDeltaInSeconds);

  }
}
