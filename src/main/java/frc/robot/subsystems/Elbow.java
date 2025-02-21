package frc.robot.subsystems;

import frc.churrolib.HardwareRegistry;
import frc.robot.Hardware;

import java.util.function.DoubleSupplier;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elbow extends SubsystemBase {

  final SparkMax m_elbowMotor = new SparkMax(Hardware.Elbow.neoMotorCAN, MotorType.kBrushless);
  final SparkClosedLoopController m_elbowPIDController = m_elbowMotor.getClosedLoopController();
  final SparkAbsoluteEncoder m_absoluteEncoder = m_elbowMotor.getAbsoluteEncoder();
  final DoubleSupplier m_elevatorHeight;

  double m_targetPosition = 0.0;

  public Elbow(DoubleSupplier elevatorHeight) {
    setDefaultCommand(stop());
    HardwareRegistry.registerHardware(m_elbowMotor);

    m_elevatorHeight = elevatorHeight;

    final SparkMaxConfig elbowConfig = new SparkMaxConfig();
    elbowConfig.absoluteEncoder
        .inverted(Hardware.Elbow.encoderIsInverted)
        .positionConversionFactor(1)
        .velocityConversionFactor(1 / 60.0);
    elbowConfig
        .inverted(Hardware.Elbow.motorIsInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Hardware.Elbow.currentLimitInAmps);
    elbowConfig.closedLoop
        .positionWrappingEnabled(false) // we don't want it to try to wrap around and break, always go up
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(Hardware.Elbow.kP)
        .i(Hardware.Elbow.kI)
        .d(Hardware.Elbow.kD)
        .outputRange(Hardware.Elbow.minOutput, Hardware.Elbow.maxOutput);

    // TODO: try maxmotion again once we reduce the gearbox
    // elbowConfig.closedLoop.maxMotion
    // .maxVelocity(1000)
    // .maxAcceleration(1000)
    // .allowedClosedLoopError(1);

    m_elbowMotor.configure(elbowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command stop() {
    return run(() -> {
      m_elbowMotor.stopMotor();
    });
  }

  public Command receive() {
    return moveToPosition(0.05);
  }

  public Command aimAtReef() {
    return moveToPosition(0.5);
  }

  public Command aimAtAlgae() {
    return moveToPosition(0.3);
  }

  public boolean isAtTarget() {
    double distanceFromTarget = Math.abs(m_elbowMotor.getAbsoluteEncoder().getPosition() - m_targetPosition);
    return distanceFromTarget < Hardware.Elbow.targetToleranceInRotations;
  }

  private Command moveToPosition(double targetPosition) {
    return run(() -> {
      double currentElevatorHeight = m_elevatorHeight.getAsDouble();
      double safeTargetPosition = targetPosition;
      if (currentElevatorHeight < 0.07) {
        // safeTargetPosition = MathUtil.clamp(targetPosition,
        // Hardware.Elbow.minRotations, 0.02);
        safeTargetPosition = MathUtil.clamp(targetPosition, Hardware.Elbow.minRotations, 0.7);
      } else {
        safeTargetPosition = MathUtil.clamp(targetPosition, 0.03, 0.45);
      }
      m_targetPosition = safeTargetPosition;
      m_elbowPIDController.setReference(m_targetPosition, ControlType.kPosition);
      boolean needsWraparoundSafetyFix = getCurrentElbowPosition() > Hardware.Elbow.maxRotations * 1.08;
      if (needsWraparoundSafetyFix) {
        m_elbowMotor.set(Hardware.Elbow.speedForResettingPosition);
      }
    });

  }

  private double getCurrentElbowPosition() {
    return m_elbowMotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elbow/TargetPosition", m_targetPosition);
    SmartDashboard.putNumber("Elbow/CurrentPosition", getCurrentElbowPosition());
    SmartDashboard.putNumber("Elbow/AppliedOutput", m_elbowMotor.getAppliedOutput());
  }
}
