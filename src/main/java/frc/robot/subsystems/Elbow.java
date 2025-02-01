package frc.robot.subsystems;

import frc.churrolib.simulation.SimulationRegistry;
import frc.robot.Hardware;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elbow extends SubsystemBase {
  final SparkMax m_elbowMotor = new SparkMax(Hardware.Elbow.neoMotorCAN, MotorType.kBrushless);
  final SparkClosedLoopController m_elbowController = m_elbowMotor.getClosedLoopController();

  public Elbow() {
    setDefaultCommand(recieve());
    SimulationRegistry.registerHardware(m_elbowMotor);
    // TODO: add PID configuration
  }

  public Command recieve() {
    return run(() -> {
      m_elbowController.setReference(0, ControlType.kMAXMotionPositionControl);
    });
  }

  public Command move1Beta() {
    return run(() -> {
      m_elbowController.setReference(0.1, ControlType.kMAXMotionPositionControl);
    });
  }

  public Command move2Sigma() {
    return run(() -> {
      m_elbowController.setReference(0.2, ControlType.kMAXMotionPositionControl);
    });
  }

  public Command move3Alpha() {
    return run(() -> {
      m_elbowController.setReference(0.3, ControlType.kMAXMotionPositionControl);
    });
  }

}
// TODO: Configure sparkMax to us absolute encoder
