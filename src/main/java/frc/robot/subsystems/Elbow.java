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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elbow extends SubsystemBase {
  final SparkMax m_elbowMotor = new SparkMax(Hardware.Elbow.neoMotorCAN, MotorType.kBrushless);
  final SparkClosedLoopController m_elbowController = m_elbowMotor.getClosedLoopController();
  final SparkMaxConfig config = new SparkMaxConfig();
  final SlewRateLimiter rateLimiter = new SlewRateLimiter(12);
  final SparkAbsoluteEncoder m_absoluteEncoder;

  final DoublePublisher m_currentPositionPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("CurrentElbowPosition")
      .publish();
  final DoublePublisher m_targetPositionPublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("TargetElbowPosition")
      .publish();

  // TODO: implement safety, this is just for testing
  final BooleanSupplier m_highEnoughToExtend = () -> true;
  final BooleanSupplier m_lowEnoughToRetract = () -> true;

  class Constants {

    static final double kMaxVelocityRadPerSecond = 3;
    static final double kMaxAccelerationRadPerSecSquared = 3;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    static final double kArmOffsetRads = 0;

    // These are all the constants from the SparkMAX demo code.
    static final double kIz = 0;
    static final double kFF = 0;
    static final double kMaxOutput = 0.9;
    static final double kMinOutput = -0.7;

    // These constants are from the sample WPIlib code and shouldn't need to change.
    static final int kCPR = 8192;

    // Default slot should be fine according to:
    // https://www.chiefdelphi.com/t/sparkmax-pid-controller/427438/4
    static final int defaultPidSlot = 0;

    static final double kP = 4.6;
    static final double kI = 0.0;
    static final double kD = 2;
  }

  public Elbow() {
    setDefaultCommand(receive());
    HardwareRegistry.registerHardware(m_elbowMotor);
    m_absoluteEncoder = m_elbowMotor.getAbsoluteEncoder();

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
      m_currentPositionPublisher.set(currentPosition);
      m_targetPositionPublisher.set(targetPosition);
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
