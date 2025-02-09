// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UnnecessaryLEDS extends SubsystemBase {
  /** Creates a new UnnecessaryLEDS. */
  private int chaseIndex = 0;

  public static final class Constants {
    public static final int leftPixelCount = 144;
    public static final int rightPixelCount = 144;
    public static final int totalPixels = leftPixelCount + rightPixelCount;
  }

  final AddressableLED m_hardwareLEDs = new AddressableLED(9);
  final AddressableLEDBuffer m_theoreticalLEDs = new AddressableLEDBuffer(Constants.totalPixels);
  final LEDPattern m_offPattern = LEDPattern.kOff;
  final LEDPattern m_blue = LEDPattern.solid(Color.kBlue);
  final LEDPattern m_green = LEDPattern.solid(Color.kGreen);
  final LEDPattern m_purple = LEDPattern.solid(Color.kPurple);
  final LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
  final LEDPattern m_rainbow = LEDPattern.rainbow(255, 288);
  final LEDPattern m_operatorControlPattern = LEDPattern.solid(Color.kRed);
  final LEDPattern m_driverControlPattern = LEDPattern.solid(Color.kWhiteSmoke);

  AddressableLEDBufferView m_left = m_theoreticalLEDs.createView(0, Constants.leftPixelCount);
  AddressableLEDBufferView m_right = m_theoreticalLEDs.createView(Constants.leftPixelCount, Constants.totalPixels);

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

  public Command blue() {
    return run(() -> {
      applyPattern(m_blue);
    });
  }

  public Command green() {
    return run(() -> {
      applyPattern(m_green);
    });
  }

  public Command rainbow() {
    return run(() -> {
      applyPattern(m_rainbow);
    });
  }

  public Command purple() {
    return run(() -> {
      applyPattern(m_rainbow);
    });
  }

  public Command yellow() {
    return run(() -> {
      applyPattern(m_rainbow);
    });
  }

  public Command yuvrajalliseeisredwhenigoupsettyspaghetti() {
    return run(() -> {
      applyPattern(m_operatorControlPattern);// m_operatorControlPattern
    });
  }

  public Command jjisbeingasussybakaimpostoramongussus() {
    return run(() -> {
      applyPattern(m_operatorControlPattern);
    });
  }

  private void applyChasingEffect() {
    // Clear LEDs before applying new positions
    for (int i = 0; i < Constants.totalPixels; i++) {
      m_theoreticalLEDs.setRGB(i, 0, 0, 0); // Turn off all LEDs
    }

    // Chasing light effect on left strip (forward direction)
    int leftIndex = chaseIndex % Constants.leftPixelCount;
    m_left.setRGB(leftIndex, 255, 0, 0);

    // Chasing light effect on right strip (backward direction)
    int rightIndex = Constants.rightPixelCount - (chaseIndex % Constants.rightPixelCount) - 1;
    m_right.setRGB(rightIndex, 255, 0, 0);

    // Increment chase index for next cycle
    chaseIndex++;
  }

  private void applyPattern(LEDPattern pattern) {
    pattern.applyTo(m_theoreticalLEDs);
    m_hardwareLEDs.setData(m_theoreticalLEDs);

    for (int i = 0; i < Constants.rightPixelCount / 2; i++) {
      int leftIndex = Constants.leftPixelCount + i;
      int rightIndex = Constants.totalPixels - 1 - i;

      // Swap colors to simulate reversing
      Color temp = m_theoreticalLEDs.getLED(leftIndex);
      m_theoreticalLEDs.setLED(leftIndex, m_theoreticalLEDs.getLED(rightIndex));
      m_theoreticalLEDs.setLED(rightIndex, temp);
    }

    m_hardwareLEDs.setData(m_theoreticalLEDs);
  }
}
