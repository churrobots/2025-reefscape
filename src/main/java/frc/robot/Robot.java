/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.churrolib.DeviceRegistry;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private RobotSimulator m_robotSimulator;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.ensureSubsystemsHaveDefaultCommands();
    m_robotContainer.bindCommandsToDriverController();
    m_robotContainer.bindCommandsToOperatorController();
    m_robotContainer.registerCommandsForUseInAutonomous();

    // Now that we've registered all the commands that Autonomous routines
    // might use, we can tell the dashboard to be built, which likely
    // includes the AutoBuilder reading from the NamedCommands global
    // that will now be populated properly.
    m_robotContainer.setupDriverStationDashboard();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationInit() {
    // Now we can instantiate the simulator since all the motors
    // and subsystems should have been instantiated by now.
    // NOTE: we don't instantiate the simulator in robotInit since
    // we don't want any simulation errors to break competition mode.
    if (m_robotSimulator == null) {
      m_robotSimulator = new RobotSimulator();
    }
  }

  @Override
  public void simulationPeriodic() {
    // TODO: use a faster loop in a thread to make ChurroSim more realistic to motor
    // controller clock speeds in the real world. The render() doesn't need to be
    // as rapid, so that's why we have iterate() separate from render().
    if (m_robotSimulator != null) {
      m_robotSimulator.iterate(TimedRobot.kDefaultPeriod);
      m_robotSimulator.render(TimedRobot.kDefaultPeriod);
    }
  }

  @Override
  public void disabledInit() {
    // We could put this back in later if we want it, removed since
    // it is generally easier to just check for state rather than
    // tracking transitions.
    // m_robotContainer.handleDisable();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.readSelectedAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
