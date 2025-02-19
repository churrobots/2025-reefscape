package frc.robot.subsystems;

import frc.churrolib.HardwareRegistry;
import frc.robot.Hardware;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elbow extends SubsystemBase {
  final SparkMax m_elbowMotor = new SparkMax(Hardware.Elbow.neoMotorCAN, MotorType.kBrushless);
  final SparkClosedLoopController m_elbowController = m_elbowMotor.getClosedLoopController();
  final SparkMaxConfig config = new SparkMaxConfig();
  final SlewRateLimiter rateLimiter = new SlewRateLimiter(2);
  final SparkAbsoluteEncoder m_absoluteEncoder;
  double m_targetPosition = 0;

  final DoublePublisher m_currentPositionPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("ElbowCurrentPosition")
      .publish();
  double m_currentPosition;
  final DoublePublisher m_targetPositionPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("ElbowTargetPosition")
      .publish();
  final DoublePublisher m_distanceFromTargetPositionPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("ElbowDistanceFromTargetPosition")
      .publish();
  final BooleanPublisher m_isSafePosition = NetworkTableInstance.getDefault()
      .getBooleanTopic("ElbowPositionIsSafe").publish();
  final DoublePublisher m_appliedOutput = NetworkTableInstance.getDefault()
      .getDoubleTopic("ElbowAppliedOutput")
      .publish();

  // TODO: implement safety, this is just for testing
  final BooleanSupplier m_lowEnoughToExtend = () -> {
    if (m_currentPosition > 0.2) {
      return false;
    } else {
      return true;
    }
  };
  final BooleanSupplier m_highEnoughToRetract = () -> {
    if (m_currentPosition < 0) {
      return false;
    } else {
      return true;
    }
  };

  class Constants {
    static final double kP = 0.3;
    static final double kI = 0.0;
    static final double kD = 0;
  }

  public Elbow() {
    setDefaultCommand(stop());
    HardwareRegistry.registerHardware(m_elbowMotor);
    m_absoluteEncoder = m_elbowMotor.getAbsoluteEncoder();
    m_currentPosition = m_elbowMotor.getAbsoluteEncoder().getPosition();

    // Setup motor and closedloop control configuration
    config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    config.absoluteEncoder.inverted(true);

    // config.encoder
    // .positionConversionFactor(1000)
    // .velocityConversionFactor(1000);

    // TODO: if needed, add config.closedLoop.maxOutput and .minOutput
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .velocityFF(.9)
        // .maxOutput(0.3)
        .pid(Constants.kP, Constants.kI, Constants.kD);

    m_elbowMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elbow/Position", m_elbowMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("Elbow/TargetPosition", m_targetPosition);
  }

  public Command stop() {
    return run(() -> {
      m_elbowMotor.stopMotor();
    });
  }

  // Base state, Driection Stright down. Called recive becasue its the intake
  // positon.
  public Command receive() {
    return moveToPosition(0);
  }

  // Position to score L1-trough which is the only one with a unique angle, all
  // the other level ones are the same
  public Command move1Beta() {
    return moveToPosition(0.1);
  }

  // Positon to score L2 & L3
  public Command move2Sigma() {
    return moveToPosition(0.2);
  }

  // Position to remove Algae
  public Command moveAlgae() {
    return moveToPosition(0.3);
  }

  private Command moveToPosition(double targetPosition) {
    boolean isExtending = targetPosition >= 0.1;
    boolean isRetracting = targetPosition < 0.1;
    return run(() -> {
      double currentPosition = m_elbowMotor.getAbsoluteEncoder().getPosition();
      this.m_currentPosition = currentPosition;
      m_currentPositionPublisher.set(currentPosition);
      m_targetPositionPublisher.set(targetPosition);
      boolean isValidEncoderReading = currentPosition < 0.7;
      boolean isSafe = isExtending && m_lowEnoughToExtend.getAsBoolean()
          || isRetracting && m_highEnoughToRetract.getAsBoolean();
      if (isSafe || true) {
        m_isSafePosition.set(true);
        m_targetPosition = targetPosition;
        m_elbowController.setReference(targetPosition, ControlType.kPosition);
      } else {
        m_isSafePosition.set(false);
      }
      m_appliedOutput.set(m_elbowMotor.getAppliedOutput());
    }).until(() -> {
      double distanceFromTarget = Math.abs(m_elbowMotor.getAbsoluteEncoder().getPosition() - targetPosition);
      m_distanceFromTargetPositionPublisher.set(distanceFromTarget);
      return distanceFromTarget < 0.02;
    }).withTimeout(15);
  }
}
