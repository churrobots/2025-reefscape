// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

/**
 * This singleton collects all simulation objects in a single place so that we
 * can update them on whatever frequency we like. Instead of overriding
 * `simulationPeriodic()` in your subsystems, you just use `ChurroSim.register`.
 */
public class DeviceRegistry {

  static DeviceRegistry instance;
  final List<TalonFX> m_talonFXList;
  final List<VictorSPX> m_victorSPXList;
  final List<SparkMax> m_sparkMaxList;
  final List<Pigeon2> m_pigeon2List;

  private DeviceRegistry() {
    m_talonFXList = new ArrayList<>();
    m_victorSPXList = new ArrayList<>();
    m_sparkMaxList = new ArrayList<>();
    m_pigeon2List = new ArrayList<>();
  }

  private static DeviceRegistry getInstance() {
    if (instance == null) {
      instance = new DeviceRegistry();
    }
    return instance;
  }

  public static void registerDevice(TalonFX device) {
    getInstance().m_talonFXList.add(device);
  }

  public static void registerDevice(VictorSPX device) {
    getInstance().m_victorSPXList.add(device);
  }

  public static void registerDevice(SparkMax device) {
    getInstance().m_sparkMaxList.add(device);
  }

  public static void registerDevice(Pigeon2 device) {
    getInstance().m_pigeon2List.add(device);
  }

  public static TalonFX getTalonFX(int canId) {
    for (TalonFX device : getInstance().m_talonFXList) {
      if (device.getDeviceID() == canId) {
        return device;
      }
    }
    return null;
  }

  public static TalonFX getVictorSPX(int canId) {
    for (TalonFX device : getInstance().m_talonFXList) {
      if (device.getDeviceID() == canId) {
        return device;
      }
    }
    return null;
  }

  public static SparkMax getSparkMax(int canId) {
    for (SparkMax device : getInstance().m_sparkMaxList) {
      if (device.getDeviceId() == canId) {
        return device;
      }
    }
    return null;
  }

  public static Pigeon2 getPigeon2(int canId) {
    for (Pigeon2 device : getInstance().m_pigeon2List) {
      if (device.getDeviceID() == canId) {
        return device;
      }
    }
    return null;
  }

}
