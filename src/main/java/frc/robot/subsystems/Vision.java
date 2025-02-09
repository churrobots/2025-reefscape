package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import swervelib.SwerveDrive;

import java.awt.Desktop;

public class Vision {
  PhotonCamera m_camera = new PhotonCamera("photonvision");
  static AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // TODO: Update this to match the camera position on the bot
  // Currently set to a camera mounted facing forward, 0.5 meters forwards of
  // center, 0.0 meters right of center, 0.5 meters up from center

  Transform3d m_robotToCam1 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_robotToCam);
  Supplier<Pose2d> m_currentPose;
  Field2d m_field;

  Camera[] m_cameras =
      // TODO: Update this to match the camera position on the bot
      // Currently set to a camera mounted facing forward, 0.5 meters forwards of
      // center, 0.0 meters right of center, 0.5 meters up from center
      { new Camera("camera 1", m_robotToCam1) };

  final StructPublisher<Pose2d> m_publisher = NetworkTableInstance.getDefault()
      .getStructTopic("VisionPose", Pose2d.struct).publish();

  // var result = camera.getLatestResult();
  // boolean hasTargets = result.hasTargets();

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
   * Update the pose estimation inside of {@link Drivetrain} with all of the given
   * poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
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
    return poseEst;
  }

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
  public void updateVisionField() {
  }

}
