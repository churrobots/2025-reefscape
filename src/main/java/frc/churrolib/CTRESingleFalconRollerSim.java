// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Simulates a roller system driven by a Falcon.
 * 
 * Inspiration taken from Quixilver, with the arm removed. Reference back
 * here if we want to figure out how to properly attach a roller to an arm.
 * https://github.com/frc604/robot-sim-example/blob/main/src/main/java/frc/robot/subsystems/IntakeSubsystem.java
 */
public class CTRESingleFalconRollerSim implements ChurroSimEntity {

  final TalonFX m_rollerMotor;
  final TalonFXSimState m_rollerMotorSim;
  final FlywheelSim m_rollerPhysicsSim;
  final Mechanism2d m_vizRoller;
  final MechanismRoot2d m_vizAxle;
  final MechanismLigament2d m_vizWheels;

  public CTRESingleFalconRollerSim(
      TalonFX rollerMotor,
      double rollerReduction,
      double rollerMOI,
      String visualizationName) {

    m_rollerMotor = rollerMotor;
    m_rollerMotorSim = m_rollerMotor.getSimState();

    // Rollers are basically flywheels.
    m_rollerPhysicsSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), rollerMOI, rollerReduction),
        DCMotor.getFalcon500(1));

    // Visualize this as a yellow box, so you can see it spinning.
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
    m_vizRoller = new Mechanism2d(1, 1);
    m_vizAxle = m_vizRoller.getRoot("Roller Axle", 0.5, 0.5);
    m_vizWheels = m_vizAxle.append(
        new MechanismLigament2d("Roller Wheels", 0.1, 0.0, 5.0, new Color8Bit(Color.kYellow)));
    SmartDashboard.putData(visualizationName, m_vizRoller);
  }

  @Override
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

    // Update viz based on sim
    double speedReductionPercentageSoSpinningIsVisibleToHumanEye = 0.03;
    m_vizWheels.setAngle(
        m_vizWheels.getAngle()
            + Math.toDegrees(m_rollerPhysicsSim.getAngularVelocityRPM())
                * timeDeltaInSeconds
                * speedReductionPercentageSoSpinningIsVisibleToHumanEye);
  }
}
