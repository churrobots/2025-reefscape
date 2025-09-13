package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

public class NEOMotor extends SubsystemBase {

  // Define the motor and its controller
  private final SparkMax m_NeoMotor;
  private final SparkClosedLoopController m_PIDController;

  // Constructor to initialize the motor
  public NEOMotor(int deviceId) {
    // Initialize the motor with the given CAN device ID
    m_NeoMotor = new SparkMax(deviceId, MotorType.kBrushless);

    // Get the PID controller for the motor
    m_PIDController = m_NeoMotor.getClosedLoopController();
  }

  // Method to make the motor spin
  public void setMotorVelocity(double velocity) {
    // Use the PID controller to set the motor velocity
    m_PIDController.setReference(velocity, ControlType.kVelocity);
  }
  
}
