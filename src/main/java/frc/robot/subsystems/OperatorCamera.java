// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.RobotBase;

public class OperatorCamera {

  UsbCamera m_camera = null;
  Thread m_highQualityVisionThread = null;

  public OperatorCamera() {

    // Camera Option 1: Microsoft Lifecam HD 3000 webcam plugged directly into
    // roboRIO over USB.
    //
    // Viewing in Elastic Dashboard:
    // Appears as CameraPublisher->"USB Camera 0".
    // Set Elastic CameraStream widget properties to FPS=30, Resolution=320x240,
    // Quality=0. Should get around 30fps.
    //
    // Notes:
    // - This camera has a hardware maximum of 30fps.
    // - There are some wavy effects on the image when moving quickly, potentially
    // due to rolling shutter?

    // Camera Option 2: Arducam OV9281 connected over USB to the OrangePi running
    // PhotonVision. OrangePi is connected over Ethernet to the robot router.
    // For now, power the OrangePi via an external USB C power brick.
    //
    // Configure camera via photonvision: http://photonvision.local:5800/#/dashboard
    // Camera Name="Churrovision".
    // Recommend resolution of either 320x240 @ 100Hz MJPEG , or 640x480 @ 100Hz
    // MJPEG
    //
    // Viewing in Elastic Dashboard:
    // Appears as CameraPublisher->"photonvision_Port_1184_Output_MJPEG_Server".
    // Set Elastic CameraStream FPS/Resolution to match whatever was configured on
    // the photonvision dashboard. At Quality=0, observed ~50fps for the 640x480,
    // ~100fps for the 320x240.
    //
    // No code needed for Option 2.

  }

  public void startLowQualityStream() {
    if (RobotBase.isReal()) {
      if (m_camera != null) {
        m_camera.close();
      }
      m_camera = CameraServer.startAutomaticCapture();
      m_camera.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);
      m_camera.setExposureManual(55);
    }
  }

  public void startHighQualityStream() {
    boolean alreadyRunning = m_highQualityVisionThread != null;
    if (!alreadyRunning) {
      m_highQualityVisionThread = new Thread(
          () -> {
            if (m_camera != null) {
              m_camera.close();
            }
            m_camera = CameraServer.startAutomaticCapture();
            m_camera.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);

            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();

            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);

            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat = new Mat();

            // This cannot be 'true'. The program will never exit if it is. This
            // lets the robot stop this thread when restarting robot code or
            // deploying.
            while (!Thread.interrupted()) {
              // Tell the CvSink to grab a frame from the camera and put it
              // in the source mat. If there is an error notify the output.
              if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
              }
              int centerX = mat.cols() / 2;
              int centerY = mat.rows() / 2;

              // Crosshair vertical line
              Imgproc.line(mat,
                  new org.opencv.core.Point(centerX, 0),
                  new org.opencv.core.Point(centerX, mat.rows()),
                  new Scalar(0, 255, 0), 2);

              // Crosshair horizontal line
              Imgproc.line(mat,
                  new org.opencv.core.Point(0, centerY),
                  new org.opencv.core.Point(mat.cols(), centerY),
                  new Scalar(0, 255, 0), 2);
              // Put a rectangle on the image
              // Imgproc.rectangle(
              // mat, new Point(10, 10), new Point(40, 40), new Scalar(255, 255, 255), 5);
              // Give the output stream a new image to display
              outputStream.putFrame(mat);
            }
          });
      m_highQualityVisionThread.setDaemon(true);
      m_highQualityVisionThread.start();
    }
  }
}
