// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UnnecessaryLEDS extends SubsystemBase {
  /** Creates a new UnnecessaryLEDS. */
  public static final class Constants {
    public static final int leftPixelCount = 144;
    public static final int rightPixelCount = 144;
    public static final int totalPixels = leftPixelCount + rightPixelCount;
  }

  final AddressableLED m_hardwareLEDs = new AddressableLED(9);
  final AddressableLEDBuffer m_theoreticalLEDs = new AddressableLEDBuffer(Constants.totalPixels);
  final LEDPattern m_offPattern = LEDPattern.kOff;
  final LEDPattern m_operatorControlPattern = LEDPattern.solid(Color.kRed);
  final LEDPattern m_driverControlPattern = LEDPattern.solid(Color.kWhiteSmoke);

  public UnnecessaryLEDS() {
    m_hardwareLEDs.setLength(m_theoreticalLEDs.getLength());
    m_hardwareLEDs.start();
    setDefaultCommand(disable());
  }

  public Command disable() {
    return run(() -> {
      applyPattern(m_offPattern);
    });
  }

  public Command yuvrajalliseeisredwhenigoupsettyspaghetti() {
    return run(() -> {
      applyPattern(m_operatorControlPattern);
    });
  }

  public Command jjisbeingasussybakaimpostoramongussus() {
    return run(() -> {
      applyPattern(m_operatorControlPattern);
    });
  }

  private void applyPattern(LEDPattern pattern) {
    pattern.applyTo(m_theoreticalLEDs);
    m_hardwareLEDs.setData(m_theoreticalLEDs);
  }
}
