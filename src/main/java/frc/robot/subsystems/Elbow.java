package frc.robot.subsystems;

import frc.churrolib.HardwareRegistry;
import frc.robot.Hardware;

import java.util.function.BooleanSupplier;

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
  final SparkMaxConfig config = new SparkMaxConfig();

  // TODO: implement safety, this is just for testing
  final BooleanSupplier m_highEnoughToExtend = () -> true;
  final BooleanSupplier m_lowEnoughToRetract = () -> true;

  class Constants {
    static final double kP = .0001;
    static final double kI = 0.0;
    static final double kD = 0.0;
  }

  public Elbow() {
    setDefaultCommand(recieve());
    HardwareRegistry.registerHardware(m_elbowMotor);

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
    return moveToPosition(0);
  }

  // Position to score L1-trough
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
    boolean isExtending = targetPosition > 0.1;
    boolean isRetracting = targetPosition < 0.2;
    return run(() -> {
      boolean isSafe = isExtending && m_highEnoughToExtend.getAsBoolean()
          || isRetracting && m_lowEnoughToRetract.getAsBoolean();
      if (isSafe) {
        m_elbowController.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
      }
    }).until(() -> {
      double distanceFromTarget = Math.abs(m_elbowMotor.getAbsoluteEncoder().getPosition() - targetPosition);
      return distanceFromTarget < 0.02;
    }).withTimeout(3);
  }
}
