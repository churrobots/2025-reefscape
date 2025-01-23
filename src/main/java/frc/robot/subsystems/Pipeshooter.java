// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.churrolib.simulation.SimulationRegistry;
import frc.robot.Hardware;

public class Pipeshooter extends SubsystemBase {

  final TalonFX m_coralMotor = new TalonFX(Hardware.Shooter.falconMotorCAN);
  final PositionDutyCycle m_feedTargetPosition = new PositionDutyCycle(5); // place to tweak later

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

    // TODO: we are getting some kind of error where it thinks we're
    // applying/refreshing config over and over again?
    // > Do not apply or refresh configs periodically, as configs are blocking.
    // > talon fx 10 ("") Config
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_coralMotor.getConfigurator().apply(config);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      // TODO: set some sticky error state and show it via LEDs?
      System.out.println("Could not apply config, error code:" +
          status.toString());
    }
    SimulationRegistry.registerHardware(m_coralMotor);
  }

  public void coralIntake() {
    m_coralMotor.set(1);
  }

  public void stopCoralIntake() {
    m_coralMotor.set(0);
    m_coralMotor.setPosition(0); // Set encoder position to zero
  }

  public void feedCoral() {
    m_coralMotor.setControl(m_feedTargetPosition);
  }

  public void shootCoral() {
    m_coralMotor.set(-1);
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