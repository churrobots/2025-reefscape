// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib.simulation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Hardware;

/** Add your docs here. */
public class CTREDoubleFalconElevatorSim {
  final TalonFX m_leaderMotor;
  final TalonFX m_followerMotor;
  final TalonFXSimState m_leaderSimState;
  final TalonFXSimState m_followerSimState;
  final ElevatorSim m_elevatorPhysicsSim;

  public CTREDoubleFalconElevatorSim(
      TalonFX leaderMotor,
      TalonFX followerMotor,
      double gearboxReduction,
      double simCarriageMass,
      double sprocketPitchDiameter,
      double minHeightInMeters,
      double maxHeightInMeters,
      double startingHeightMeters) {

    m_leaderMotor = leaderMotor;
    m_followerMotor = followerMotor;

    m_leaderSimState = leaderMotor.getSimState();
    m_followerSimState = followerMotor.getSimState();

    m_elevatorPhysicsSim = new ElevatorSim(
        DCMotor.getFalcon500(2),
        gearboxReduction,
        simCarriageMass,
        // TODO: figure out why Quixilver halved the pitch diameter?
        sprocketPitchDiameter * 0.5,
        minHeightInMeters,
        maxHeightInMeters,
        true,
        startingHeightMeters);
  }

  public double getElevatorPositionInMeters() {
    return m_elevatorPhysicsSim.getPositionMeters();
  }

  public void iterate(double timeDeltaInSeconds) {
    double leaderOutputVoltage = m_leaderMotor.get() * RobotController.getBatteryVoltage();
    double followerOutputVoltage = m_followerMotor.get() * RobotController.getBatteryVoltage();
    // m_elevatorPhysicsSim.setInput(leaderOutputVoltage, followerOutputVoltage);
    m_elevatorPhysicsSim.setInput(leaderOutputVoltage);
    m_elevatorPhysicsSim.update(timeDeltaInSeconds);

    double metersPerSecond = m_elevatorPhysicsSim.getVelocityMetersPerSecond();
    double rotationsPerSecond = metersPerSecond / (Hardware.Elevator.sprocketPitchDiameter * Math.PI);
    m_leaderSimState.setRotorVelocity(rotationsPerSecond);
    double positionInMeters = m_elevatorPhysicsSim.getPositionMeters();
    double positionInRotations = positionInMeters / (Hardware.Elevator.sprocketPitchDiameter * Math.PI);
    m_leaderSimState.setRawRotorPosition(positionInRotations);
    SmartDashboard.putNumber("Elevator Velocity", rotationsPerSecond);
    SmartDashboard.putNumber("Elevator Rotations", positionInRotations);
    // TODO: what about the follower?
  }

}
