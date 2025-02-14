package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;

public class Camera {
  public PhotonPoseEstimator m_photonPoseEstimator;
  PhotonCamera m_photonCamera;
  Transform3d m_robotToCam;

  // For simulation
  public PhotonCameraSim m_cameraSim;

  /**
   * Construct a Photon Camera class with help. Standard deviations are fake
   * values, experiment and determine
   * estimation noise on an actual robot.
   *
   * @param name       Name of the PhotonVision camera found in the PV
   *                   UI.
   * @param robotToCam {@link Transform3d} relative to the center of
   *                   the robot.
   */
  Camera(String name, Transform3d robotToCam) {
    m_photonCamera = new PhotonCamera(name);
    m_robotToCam = robotToCam;
    // Construct PhotonPoseEstimator
    m_photonPoseEstimator = new PhotonPoseEstimator(Vision.m_aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_robotToCam);

    if (Robot.isSimulation()) {
      SimCameraProperties cameraProp = new SimCameraProperties();
      // A 640 x 480 camera with a 100 degree diagonal FOV.
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
      // Approximate detection noise with average and standard deviation error in
      // pixels.
      cameraProp.setCalibError(0.25, 0.08);
      // Set the camera image capture framerate (Note: this is limited by robot loop
      // rate).
      cameraProp.setFPS(30);
      // The average and standard deviation in milliseconds of image data latency.
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);
      m_cameraSim = new PhotonCameraSim(m_photonCamera, cameraProp);
      m_cameraSim.enableDrawWireframe(true);

    }

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
    if (allResults.size() == 0) {
      return Optional.empty();
    }
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
    Optional<PhotonPipelineResult> latestResult = getLatestResult();
    if (!latestResult.isPresent()) {
      return Optional.empty();
    }
    return m_photonPoseEstimator.update(latestResult.get());
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

  /**
   * Add camera to {@link VisionSystemSim} for simulated photon vision.
   *
   * @param systemSim {@link VisionSystemSim} to use.
   */
  public void addToVisionSim(VisionSystemSim systemSim) {
    if (Robot.isSimulation()) {
      systemSim.addCamera(m_cameraSim, m_robotToCam);
    }
  }

}
