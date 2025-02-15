package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Hardware;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Vision {
  PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, Hardware.Vision.robotToCam1);
  Camera[] m_cameras = {
      new Camera("camera 1", Hardware.Vision.robotToCam1, VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)) };

  static AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Supplier<Pose2d> m_currentPose;
  Field2d m_field;
  final StructPublisher<Pose2d> m_publisher = NetworkTableInstance.getDefault()
      .getStructTopic("VisionPose", Pose2d.struct).publish();

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

    for (Camera camera : m_cameras) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        m_currentPose = () -> pose.estimatedPose.toPose2d();
        m_publisher.set(pose.estimatedPose.toPose2d());
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
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
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

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = m_aprilTagFieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + m_aprilTagFieldLayout.toString());
    }

  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance, or -1 if distance could not be determined.
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = m_aprilTagFieldLayout.getTagPose(id);
    if (tag.isPresent()) {
      double distance = PhotonUtils.getDistanceToPose(m_currentPose.get(), tag.get().toPose2d());
      return distance;
    }
    return -1;
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Camera camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField() {
    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Camera c : m_cameras) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (m_aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = m_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    m_field.getObject("tracked targets").setPoses(poses);
  }
}
