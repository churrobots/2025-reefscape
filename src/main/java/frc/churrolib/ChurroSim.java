// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * This singleton collects all simulation objects in a single place so that we
 * can update them on whatever frequency we like. Instead of overriding
 * `simulationPeriodic()` in your subsystems, you just use `ChurroSim.register`.
 */
public class ChurroSim {

  static ChurroSim instance;
  final List<ChurroSimEntity> m_entities;
  final List<TalonFX> m_talonFXList;
  final List<VictorSPX> m_victorSPXList;

  private ChurroSim() {
    m_entities = new ArrayList<>();
    m_talonFXList = new ArrayList<>();
    m_victorSPXList = new ArrayList<>();
  }

  private static ChurroSim getInstance() {
    if (instance == null) {
      instance = new ChurroSim();
    }
    return instance;
  }

  public static void registerDevice(TalonFX device) {
    getInstance().m_talonFXList.add(device);
  }

  public static void registerDevice(VictorSPX device) {
    getInstance().m_victorSPXList.add(device);
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

  public static void registerEntity(ChurroSimEntity entity) {
    getInstance().m_entities.add(entity);
  }

  public static void iterate(double timeDeltaInSeconds) {
    for (ChurroSimEntity entity : getInstance().m_entities) {
      entity.iterate(timeDeltaInSeconds);
    }
  }
}
