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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.Robot;

public class Camera {
  public PhotonPoseEstimator m_photonPoseEstimator;
  PhotonCamera m_photonCamera;
  Transform3d m_robotToCam;
  public List<PhotonPipelineResult> resultsList = new ArrayList<>();
  private double                       lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
  public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
  public Matrix<N3, N1> curStdDevs;
  private final Matrix<N3, N1>               singleTagStdDevs = VecBuilder.fill(4, 4, 8); 
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
        // Strategies that work:
        // - CLOSEST_TO_REFERENCE_POSE
        // - CLOSEST_TO_CAMERA_HEIGHT
        // - AVERAGE_BEST_TARGETS
        // Strategies that do NOT work:
        // - CLOSEST_TO_LAST_POSE
        // - LOWEST_AMBIGUITY
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_robotToCam);
    m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

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

  private void updateUnreadResults()
  {
    double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
    double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
    double debounceTime = Milliseconds.of(15).in(Seconds);
    for (PhotonPipelineResult result : resultsList) {
      mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
    }

    resultsList = Robot.isReal() ? m_photonCamera.getAllUnreadResults() : m_cameraSim.getCamera().getAllUnreadResults();
    lastReadTimestamp = currentTimestamp;
    resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
      return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
    });
    if (!resultsList.isEmpty()) {
      updateEstimatedGlobalPose();
    }

  }
    
  private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets)
  {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = singleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
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
    }
  }

  public Optional<PhotonPipelineResult> getBestResult()
    {
      if (resultsList.isEmpty())
      {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult       = resultsList.get(0);
      double               amiguity         = bestResult.getBestTarget().getPoseAmbiguity();
      double               currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList)
      {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0)
        {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

  private void updateEstimatedGlobalPose()
    {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList)
      {
        visionEst = m_photonPoseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
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

  /**
   * Set the camera to driver mode.
   * 
   * @param driverMode True if camera should be set to driver mode.
   */
  public void setDriverMode(Boolean driverMode) {
    m_photonCamera.setDriverMode(driverMode);
  }

}
