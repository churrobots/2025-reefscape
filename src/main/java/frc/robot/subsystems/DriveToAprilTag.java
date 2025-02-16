package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Hardware;

/* Subsystem for driving to the closest AprilTag.
 * See similar code at https://github.com/FRC4131/FRCCrescendo2024/blob/b2d4fd98586f271ee1d37fbf88d8c887938df621/src/main/java/frc/robot/commands/GoToPoseTeleopCommand.java.
 */
public class DriveToAprilTag extends Command {
  Drivetrain m_drivetrain;
  Vision m_vision;

  PIDController m_turningPidController;
  PIDController m_drivingXPidController;
  PIDController m_drivingYPidController;
  Map<Integer, Number> m_reefTargetsMap;

  final IntegerPublisher m_tagIdPublisher = NetworkTableInstance.getDefault()
      .getIntegerTopic("ClosestTag.Id")
      .publish();
  final DoublePublisher m_tagDistancePublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("ClosestTag.Distance")
      .publish();
  final DoublePublisher m_tagAnglePublisher = NetworkTableInstance.getDefault()
      .getDoubleTopic("ClosestTag.DesiredAngle")
      .publish();

  public DriveToAprilTag(Drivetrain drivetrain,
      Vision vision) {

    m_drivetrain = drivetrain;
    m_vision = vision;
    m_drivingXPidController = new PIDController(0.95, 0, 0.2);
    m_drivingYPidController = new PIDController(0.95, 0, 0.2);
    m_drivingXPidController.disableContinuousInput();
    m_drivingYPidController.disableContinuousInput();
    m_turningPidController = new PIDController(0.5, 0, 0);
    m_turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_drivetrain);

    boolean isBlueAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue;
    if (isBlueAlliance) {
      m_reefTargetsMap = Hardware.Vision.blueReefTargets;
    } else {
      m_reefTargetsMap = Hardware.Vision.redReefTargets;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private Optional<Integer> getClosestTarget() {
    ArrayList<Integer> seeableTargets = new ArrayList<Integer>();
    for (Integer id : m_reefTargetsMap.keySet()) {
      if (m_vision.canSeeTarget(id)) {
        seeableTargets.add(id);
      }
    }
    if (seeableTargets.isEmpty()) {
      return Optional.empty();
    }

    Integer closestId = 0;
    Double closestDistance = Double.MAX_VALUE;
    for (Integer id : seeableTargets) {
      Double tagDistance = m_vision.getDistanceFromAprilTag(id);
      if (m_vision.getDistanceFromAprilTag(id) < closestDistance) {
        closestId = id;
        closestDistance = tagDistance;
      }
    }
    return Optional.of(closestId);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Grab the closest AprilTag.
    Optional<Integer> closestTag = getClosestTarget();
    if (closestTag.isEmpty()) {
      return;
    }
    m_tagIdPublisher.set(closestTag.get());

    // Find the target robot angle for the tag.
    Double targetAngle = m_reefTargetsMap.get(closestTag.get()).doubleValue() * Math.PI / 180;
    m_tagAnglePublisher.set(targetAngle);
    m_turningPidController.setSetpoint(targetAngle);
    Double pidAdjustedAngle = m_turningPidController.calculate(m_drivetrain.getPose().getRotation().getRadians());

    // Find the distance to the tag.
    double distanceFromAprilTag = m_vision.getDistanceFromAprilTag(closestTag.get());
    Rotation2d yaw = m_vision.getYawFromAprilTag(closestTag.get());
    m_tagDistancePublisher.set(distanceFromAprilTag);
    if (distanceFromAprilTag == -1) {
      return;
    }
    double translationX = 0;
    double translationY = 0;
    if (distanceFromAprilTag < 1) {
      // When we're up close, only adjust the angle and don't drive forward anymore.
      m_drivetrain.driveRobotRelative(0.0, 0.0, pidAdjustedAngle * 0.5);
    } else {
      m_drivingXPidController.setSetpoint(0);
      m_drivingYPidController.setSetpoint(0);
      // Flip the translations so that the PIDs make the distance go down to 0.
      translationX = yaw.getCos() * distanceFromAprilTag * -1;
      translationY = yaw.getSin() * distanceFromAprilTag * -1;
      Double pidAdjustedX = m_drivingXPidController.calculate(translationX);
      Double pidAdjustedY = m_drivingYPidController.calculate(translationY);
      m_drivetrain.driveRobotRelative(pidAdjustedX, pidAdjustedY, pidAdjustedAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    resetPublishers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    resetPublishers();
    return false;
  }

  // Clears out the NetworkTable publishers.
  void resetPublishers() {
    m_tagIdPublisher.set(0);
    m_tagDistancePublisher.set(0.0);
    m_tagAnglePublisher.set(0.0);
  }
}
