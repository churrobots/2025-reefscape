package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Hardware;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Vision {
  Camera[] m_cameras = {
      new Camera("OperatorCamera1", Hardware.Vision.robotToCamOperator),
      new Camera("VisionCameraFront", Hardware.Vision.robotToCamFront),
      new Camera("VisionCameraBack", Hardware.Vision.robotToCamBack),
  };

  static AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Supplier<Pose2d> m_currentPose;
  Field2d m_field;

  final ArrayList<StructPublisher<Pose2d>> m_posePublishers = new ArrayList<>(
      Arrays.asList(NetworkTableInstance.getDefault()
          .getStructTopic("OperatorCamera1Pose", Pose2d.struct).publish(),
          NetworkTableInstance.getDefault()
              .getStructTopic("VisionCameraFrontPose", Pose2d.struct).publish(),
          NetworkTableInstance.getDefault()
              .getStructTopic("VisionCameraBackPose", Pose2d.struct).publish()));

  // For simulation
  VisionSystemSim m_visionSim;
  // To see camera streams:
  // Go to
  // * http://localhost:1181/ <-- camera 1 raw stream
  // * http://localhost:1182/ <-- camera 1 processed stream
  // * http://localhost:1183/ <-- camera 2 raw stream
  // * http://localhost:1184/ <-- camera 2 processed stream

  // See example reference at
  // https://github.com/BroncBotz3481/YAGSL-Example/blob/main/src/main/java/frc/robot/subsystems/swervedrive/Vision.java
  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference
   *                    {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link Drivetrain#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    m_currentPose = currentPose;
    m_field = field;

    if (Robot.isSimulation()) {
      m_visionSim = new VisionSystemSim("Vision");
      m_visionSim.addAprilTags(m_aprilTagFieldLayout);

      for (Camera c : m_cameras) {
        c.addToVisionSim(m_visionSim);
      }
    }
  }

  /**
   * Update the pose estimation inside of {@link Drivetrain} with all of the given
   * poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for
       * factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate
       * pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the
       * simulator when updating the vision simulation during the simulation.
       */
      m_visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
      m_field.getObject("VisionPose").setPose(swerveDrive.getSimulationDriveTrainPose().get());
    }

    for (int i = 0; i < m_cameras.length; i++) {
      Camera camera = m_cameras[i];
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        m_currentPose = () -> pose.estimatedPose.toPose2d();
        m_posePublishers.get(i).set(pose.estimatedPose.toPose2d());
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
            pose.timestampSeconds);
      }
    }
  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   * <li>No Pose Estimates could be generated</li>
   * <li>The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and
   *         targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose(m_currentPose.get());
    if (Robot.isSimulation()) {
      Field2d debugField = m_visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est -> debugField
              .getObject("VisionEstimation")
              .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }
}
