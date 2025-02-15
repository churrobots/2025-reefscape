package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.Robot;

public class Camera {
  PhotonCamera m_photonCamera;
  Transform3d m_robotToCam;

  /**
   * Each camera holds its own pose estimation.
   */
  public PhotonPoseEstimator m_photonPoseEstimator;
  public Optional<EstimatedRobotPose> m_estimatedRobotPose = Optional.empty();

  /**
   * Use stddevs to make vision estimates more accurate.
   */
  public Matrix<N3, N1> m_curStdDevs;
  private final Matrix<N3, N1> m_defaultTagStdDevs;
  private final Matrix<N3, N1> m_multiTagStdDevs;

  /**
   * Results list to be updated periodically and cached to avoid unnecessary
   * queries and slow down from data fetches.
   */
  public List<PhotonPipelineResult> resultsList = new ArrayList<>();
  private double m_lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

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
  Camera(String name, Transform3d robotToCam, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
    m_photonCamera = new PhotonCamera(name);
    m_robotToCam = robotToCam;
    // Construct PhotonPoseEstimator
    m_photonPoseEstimator = new PhotonPoseEstimator(Vision.m_aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_robotToCam);

    this.m_defaultTagStdDevs = singleTagStdDevs;
    this.m_multiTagStdDevs = multiTagStdDevsMatrix;

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
  public Optional<PhotonPipelineResult> getBestResult() {
    if (resultsList.isEmpty()) {
      return Optional.empty();
    }

    PhotonPipelineResult bestResult = resultsList.get(0);
    double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
    double currentAmbiguity = 0;
    for (PhotonPipelineResult result : resultsList) {
      currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
      if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
        bestResult = result;
        amiguity = currentAmbiguity;
      }
    }
    return Optional.of(bestResult);
  }

  /**
   * Get the latest result from the current cache.
   *
   * @return Empty optional if nothing is found. Latest result if something is
   *         there.
   */
  public Optional<PhotonPipelineResult> getLatestResult() {
    // List<PhotonPipelineResult> allResults = m_photonCamera.getAllUnreadResults();
    // if (allResults.size() == 0) {
    // return Optional.empty();
    // }
    // return Optional.of(allResults.get(allResults.size() - 1));
    return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
  }

  /**
   * Get the estimated robot pose. Updates the current robot pose estimation,
   * standard deviations, and flushes the
   * cache of results.
   *
   * @return Estimated pose.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    // m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    // Optional<PhotonPipelineResult> latestResult = getLatestResult();
    // if (!latestResult.isPresent()) {
    // return Optional.empty();
    // }
    // return m_photonPoseEstimator.update(latestResult.get());
    updateUnreadResults();
    return m_estimatedRobotPose;
  }

  /**
   * Update the latest results, cached with a maximum refresh rate of 1req/15ms.
   * Sorts the list by timestamp.
   */
  private void updateUnreadResults() {
    double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
    double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
    // Get new results every 15ms.
    double debounceTime = Milliseconds.of(15).in(Seconds);
    for (PhotonPipelineResult result : resultsList) {
      mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
    }
    if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
        (currentTimestamp - m_lastReadTimestamp) >= debounceTime) {
      resultsList = Robot.isReal() ? m_photonCamera.getAllUnreadResults()
          : m_cameraSim.getCamera().getAllUnreadResults();
      m_lastReadTimestamp = currentTimestamp;
      resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
        return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
      });
      if (!resultsList.isEmpty()) {
        updateEstimatedGlobalPose();
      }
    }
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
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : resultsList) {
      visionEst = m_photonPoseEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
    }
    m_estimatedRobotPose = visionEst;
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
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      m_curStdDevs = m_defaultTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = m_defaultTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = m_photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        m_curStdDevs = m_defaultTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
          estStdDevs = m_multiTagStdDevs;
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        m_curStdDevs = estStdDevs;
      }
    }
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
