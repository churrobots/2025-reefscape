package frc.robot.subsystems;

import frc.churrolib.simulation.SimulationRegistry;
import frc.robot.Hardware;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elbow extends SubsystemBase {
  final SparkMax m_elbowMotor = new SparkMax(Hardware.Elbow.neoMotorCAN, MotorType.kBrushless);
  final SparkClosedLoopController m_elbowController = m_elbowMotor.getClosedLoopController();
  SparkMaxConfig config = new SparkMaxConfig();

  class Constants {
    static final double kP = .0001;
    static final double kI = 0.0;
    static final double kD = 0.0;
  }

  public Elbow() {
    setDefaultCommand(recieve());
    SimulationRegistry.registerHardware(m_elbowMotor);

    // Setup motor and closedloop control configuration
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake);

    // config.encoder
    // .positionConversionFactor(1000)
    // .velocityConversionFactor(1000);

    // TODO: if needed, add config.closedLoop.maxOutput and .minOutput
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(Constants.kP, Constants.kI, Constants.kD);

    m_elbowMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Base state, Driection Stright down. Called recive becasue its the intake
  // positon.
  public Command recieve() {
    return run(() -> {
      m_elbowController.setReference(0, ControlType.kMAXMotionPositionControl);
    });
  }

  // Position to score L1-trough
  public Command move1Beta() {
    return run(() -> {
      m_elbowController.setReference(0.1, ControlType.kMAXMotionPositionControl);
    });
  }

  // Positon to score L2 & L3
  public Command move2Sigma() {
    return run(() -> {
      m_elbowController.setReference(0.2, ControlType.kMAXMotionPositionControl);
    });
  }

  // Position to remove Algae
  public Command moveAlgae() {
    return run(() -> {
      m_elbowController.setReference(0.3, ControlType.kMAXMotionPositionControl);
    });
  }

}
