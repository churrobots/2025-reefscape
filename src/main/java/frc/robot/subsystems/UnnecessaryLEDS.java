// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Hardware;

public class UnnecessaryLEDS extends SubsystemBase {

  final AddressableLED m_hardwareLEDs = new AddressableLED(Hardware.LEDLights.ledPWM);

  final AddressableLEDBuffer m_pixels = new AddressableLEDBuffer(
      Hardware.LEDLights.leftLEDCount + Hardware.LEDLights.rightLEDCount);

  final AddressableLEDBufferView m_leftPixels = m_pixels
      .createView(0, Hardware.LEDLights.leftLEDCount - 1);

  final AddressableLEDBufferView m_rightPixels = m_pixels
      .createView(Hardware.LEDLights.leftLEDCount,
          Hardware.LEDLights.leftLEDCount + Hardware.LEDLights.rightLEDCount - 1)
      .reversed();

  final LEDPattern m_offPattern = LEDPattern.kOff;
  final LEDPattern m_disabledPattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kPurple);
  final LEDPattern m_blue = LEDPattern.solid(Color.kBlue);
  final LEDPattern m_red = LEDPattern.solid(Color.kRed);
  final LEDPattern m_green = LEDPattern.solid(Color.kGreen);
  final LEDPattern m_purple = LEDPattern.solid(Color.kPurple);
  final LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
  final LEDPattern m_deepPink = LEDPattern.solid(Color.kDeepPink);
  final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);
  final LEDPattern m_operatorControlPattern = LEDPattern.solid(Color.kRed);
  final LEDPattern m_driverControlPattern = LEDPattern.solid(Color.kWhiteSmoke);

  public UnnecessaryLEDS() {
    m_hardwareLEDs.setLength(m_pixels.getLength());
    m_hardwareLEDs.start();
    setDefaultCommand(disable());
    applyPattern(m_disabledPattern);
  }

  public Command disable() {
    return runPattern(m_disabledPattern);
  }

  public Command blue() {
    return runPattern(m_blue);
  }

  public Command red() {
    return runPattern(m_red);
  }

  public Command deepPink() {
    return runPattern(m_deepPink);
  }

  public Command green() {
    return runPattern(m_green);
  }

  public Command rainbow() {
    return runPattern(m_rainbow);
  }

  public Command purple() {
    return runPattern(m_rainbow);
  }

  public Command yellow() {
    return runPattern(m_rainbow);
  }

  public Command yuvrajalliseeisredwhenigoupsettyspaghetti() {
    return runPattern(m_operatorControlPattern);
  }

  public Command jjisbeingasussybakaimpostoramongussus() {
    return runPattern(m_operatorControlPattern);
  }

  private Command runPattern(LEDPattern pattern) {
    return run(() -> {
      applyPattern(pattern);
    }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  private void applyPattern(LEDPattern pattern) {
    LEDPattern dimmerPattern = pattern.atBrightness(Percent.of(15));
    dimmerPattern.applyTo(m_leftPixels);
    dimmerPattern.applyTo(m_rightPixels);
  }

  @Override
  public void periodic() {
    // Note: when the robot is disabled, it will not be running the
    // default command, since commands do not run until it's enabled.
    // By setting the data in periodic(), we can have lights when
    // the robot is still disabled.
    m_hardwareLEDs.setData(m_pixels);
  }
}
