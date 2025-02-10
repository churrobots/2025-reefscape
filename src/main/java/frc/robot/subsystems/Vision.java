package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Vision {
  // TODO: Update the Transform3d to match the camera position on the bot
  // Currently it is set to a camera mounted facing forward, 0.5 meters forwards
  // of center, 0.0 meters right of center, 0.5 meters up from center
  Transform3d m_robotToCam1 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_robotToCam1);
  Camera[] m_cameras = { new Camera("camera 1", m_robotToCam1) };

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

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  // public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
  // }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  // public double getDistanceFromAprilTag(int id) {
  // }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  // public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
  // }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  // public void updateVisionField() {
  // }
}
