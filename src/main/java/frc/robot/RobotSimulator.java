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
import frc.churrolib.HardwareRegistry;
import frc.churrolib.simulation.CTREDoubleFalconElevatorSim;
import frc.churrolib.simulation.CTRESingleFalconRollerSim;

public class RobotSimulator {

  final CTRESingleFalconRollerSim m_shooterSim;

  final CTREDoubleFalconElevatorSim m_elevatorSim;

  // TODO: look at Quixlib Link2d helper
  // final Link2d m_vizElevator;
  final Mechanism2d m_vizRoller;
  final MechanismRoot2d m_vizAxle;
  final MechanismLigament2d m_vizWheels;

  public RobotSimulator() {
    TalonFX coralMotor = HardwareRegistry.getTalonFX(Hardware.Pipeshooter.falconMotorCAN);
    m_shooterSim = new CTRESingleFalconRollerSim(
        coralMotor, Hardware.Pipeshooter.gearboxReduction, Hardware.Pipeshooter.simMomentOfInertia);

    m_elevatorSim = new CTREDoubleFalconElevatorSim(
        HardwareRegistry.getTalonFX(Hardware.Elevator.leaderFalconMotorCAN),
        HardwareRegistry.getTalonFX(Hardware.Elevator.followerFalconMotorCAN),
        Hardware.Elevator.gearboxReduction,
        Hardware.Elevator.simCarriageMass,
        Hardware.Elevator.sprocketPitchDiameter * 0.5,
        Hardware.Elevator.minHeightInMeters,
        Hardware.Elevator.maxHeightInMeters);

    // Setup visualizations
    // Visualize this as a yellow box, so you can see it spinning.
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
    m_vizRoller = new Mechanism2d(1, 1);
    m_vizAxle = m_vizRoller.getRoot("Roller Axle", 0.5, 0.5);
    m_vizWheels = m_vizAxle
        .append(new MechanismLigament2d("Roller Wheels", 0.1, 0.0, 5.0, new Color8Bit(Color.kYellow)));
    SmartDashboard.putData("TestShooter", m_vizRoller);
  }

  public void iterate(double timeDeltaInSeconds) {
    m_shooterSim.iterate(timeDeltaInSeconds);
    m_elevatorSim.iterate(timeDeltaInSeconds);
  }

  public void render(double timeDeltaInSeconds) {
    // Update viz based on sim
    // TODO: try to use color to indicate speed, the "scaling" really messes with
    // the low speeds, and we really just want to indicate when the speed is so fast
    // that aliasing will occur. maybe scale only above a certain velocity?
    double speedReductionPercentageSoSpinningIsVisibleToHumanEye = 0.03;
    m_vizWheels.setAngle(
        m_vizWheels.getAngle()
            + Math.toDegrees(m_shooterSim.rollerOutputVelocityRPM())
                * timeDeltaInSeconds
                * speedReductionPercentageSoSpinningIsVisibleToHumanEye);

  }
}
