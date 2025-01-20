// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.churrolib.CTRESingleFalconRollerSim;
import frc.churrolib.ChurroSim;
import frc.churrolib.ChurroSimEntity;

/** Add your docs here. */
public class RobotSimulator {

  final CTRESingleFalconRollerSim m_shooterSim;
  final Mechanism2d m_vizRoller;
  final MechanismRoot2d m_vizAxle;
  final MechanismLigament2d m_vizWheels;

  // Visualize this as a yellow box, so you can see it spinning.
  // https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html

  public RobotSimulator() {
    TalonFX coralMotor = ChurroSim.getTalonFX(Hardware.Shooter.falconMotorCAN);
    m_shooterSim = new CTRESingleFalconRollerSim(
        coralMotor, Hardware.Shooter.gearboxReduction, Hardware.Shooter.simMomentOfInertia);

    m_vizRoller = new Mechanism2d(1, 1);
    m_vizAxle = m_vizRoller.getRoot("Roller Axle", 0.5, 0.5);
    m_vizWheels = m_vizAxle
        .append(new MechanismLigament2d("Roller Wheels", 0.1, 0.0, 5.0, new Color8Bit(Color.kYellow)));

    SmartDashboard.putData("TestShooter", m_vizRoller);
    ChurroSim.registerEntity(m_shooterSim);
  }

  public void render(double timeDeltaInSeconds) {
    // Update viz based on sim
    double speedReductionPercentageSoSpinningIsVisibleToHumanEye = 0.03;
    m_vizWheels.setAngle(
        m_vizWheels.getAngle()
            + Math.toDegrees(m_shooterSim.rollerOutputVelocityRPM())
                * timeDeltaInSeconds
                * speedReductionPercentageSoSpinningIsVisibleToHumanEye);
  }
}
