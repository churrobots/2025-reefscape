package frc.robot.subsystems;

import frc.robot.Hardware;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elbow extends SubsystemBase {
  final SparkMax m_elbowMotor = new SparkMax(Hardware.Elbow.neoMotorCAN, MotorType.kBrushless);

}
// TODO: Configure sparkMax to us absolute encoder
