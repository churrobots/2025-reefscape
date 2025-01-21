// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.SimulationRegistry;
import frc.robot.Hardware;

public class Pipeshooter extends SubsystemBase {

  public Pipeshooter() {
    setDefaultCommand(new RunCommand(this::stopCoralIntake, this));
    var config = new TalonFXConfiguration();
    // TODO: consider adding these Phoenix6 equivalents of our old Phoenix5 config
    // config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.CurrentLimits.SupplyCurrentLimit = 10;
    // config.CurrentLimits.SupplyCurrentThreshold = 15;
    // config.CurrentLimits.SupplyTimeThreshold = 0.5;

    // These PID values suit a standard Falcon500, according to official example.
    // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/6be53802b071f84fd45aeca23737345cfc421072/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java#L51-L54
    config.Slot0.kP = 10;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kV = 0;
    config.Voltage.PeakForwardVoltage = 8;
    config.Voltage.PeakReverseVoltage = -8;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = coralMotor.getConfigurator().apply(config);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      // TODO: set some sticky error state and show it via LEDs?
      System.out.println("Could not apply config, error code:" +
          status.toString());
    }
    if (!status.isOK()) {
      // TODO: set some sticky error state and show it via LEDs?
      System.out.println("Could not apply config, error code:" +
          status.toString());
    }
    // TODO: this doesn't seem to target the same velocity, might have to do it
    // manually?
    // bottomMotor.setControl(new Follower(topMotor.getDeviceID(), false));
    SimulationRegistry.registerHardware(coralMotor);
  }

  public static final class Constants {
    public static final double targetVelocityTolerance = 1.5;

    public static final double bottomSpeakerVelocity = 60; // 50
    public static final double bottomSpeakerPosition = 0;
  }

  // Daniel does not have churrobot spirit
  final TalonFX coralMotor = new TalonFX(Hardware.Shooter.falconMotorCAN);

  final PositionDutyCycle feedTarget = new PositionDutyCycle(5); // place to tweak later

  public void coralIntake() {
    coralMotor.set(1);
  }

  public void stopCoralIntake() {
    coralMotor.set(0);
    coralMotor.setPosition(0); // Set encoder position to zero
  }

  public void feedCoral() {
    coralMotor.setControl(feedTarget);
  }

  public void shootCoral() {
    coralMotor.set(-1);
  }

  boolean isMotorAtTarget(TalonFX motor) {
    ControlRequest appliedControl = motor.getAppliedControl();
    if (appliedControl.getName() == "NeutralOut") {
      return false;
    } else if (appliedControl.getName() == "VelocityVoltage") {
      VelocityVoltage target = (VelocityVoltage) appliedControl;
      var actualVelocity = motor.getVelocity().getValueAsDouble();
      var expectedVelocity = target.Velocity;
      var isReversing = expectedVelocity < 0;
      if (isReversing) {
        // don't consider reversing (like keeping the note in the intake) as "at target"
        return false;
      }
      var minVelocity = expectedVelocity - Constants.targetVelocityTolerance;
      var maxVelocity = expectedVelocity + Constants.targetVelocityTolerance;
      if (actualVelocity > minVelocity && actualVelocity < maxVelocity) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  void _debug() {
    var actualBottomVelocity = coralMotor.getVelocity().getValueAsDouble();
    var actualBottomPosition = coralMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("bottomPosition", actualBottomPosition);
    SmartDashboard.putNumber("bottomVelocity", actualBottomVelocity);
  }

  @Override
  public void periodic() {
    // TODO: uncomment this to tune the shooter
    // _debug();
  }
}
// motor reference, get encoder readings, motor declared, figure how to read,
// figure how to set position/velocity
// 4 functions: intake, stop, feed, shoot
// What's done: Motor declared, set position(Check with Matt),