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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.SwerveDrive;

import java.awt.Desktop;

public class Vision {
  PhotonCamera m_camera = new PhotonCamera("photonvision");
  static AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center.
  Transform3d m_robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  // example included camera but quick fix said no
  PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_robotToCam);
  Supplier<Pose2d> m_currentPose;
  Field2d m_field;

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
  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
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
  public void updateVisionField() {
  }

  /**
   * Camera Enum to select each camera
   */
  enum Cameras {
    // NEED TO UPDATE THIS currently facing forward
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    CAM_1("camera 1", new Rotation3d(0, 0, 0), new Translation3d(0.5, 0.0, 0.5));

    public PhotonPoseEstimator m_photonPoseEstimator;
    PhotonCamera m_photonCamera;
    Transform3d m_robotToCam;

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake
     * values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV
     *                              UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of
     *                              the robot.
     */
    Cameras(String name, Rotation3d robotToCamRotation,
        Translation3d robotToCamTranslation) {
      // Forward Camera
      m_photonCamera = new PhotonCamera(name);
      m_robotToCam = new Transform3d(robotToCamTranslation, robotToCamRotation);

      // Construct PhotonPoseEstimator
      m_photonPoseEstimator = new PhotonPoseEstimator(Vision.m_aprilTagFieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_robotToCam);
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within
     * the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target.
     *         This is not the most recent result!
     */
    // public Optional<PhotonPipelineResult> getBestResult() {
    // }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is
     *         there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      List<PhotonPipelineResult> allResults = m_photonCamera.getAllUnreadResults();
      return Optional.of(allResults.get(allResults.size() - 1));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation,
     * standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return m_photonPoseEstimator.update(getLatestResult().get());
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms.
     * Sorts the list by timestamp.
     */
    private void updateUnreadResults() {
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should only be called once
     * per loop.
     *
     * <p>
     * Also includes updates for the standard deviations, which can (optionally) be
     * retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets used for
     *         estimation.
     */
    private void updateEstimatedGlobalPose() {
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

    }
  }

}
