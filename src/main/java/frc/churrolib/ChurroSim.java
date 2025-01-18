// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import java.util.ArrayList;
import java.util.List;

/**
 * This singleton collects all simulation objects in a single place so that we
 * can update them on whatever frequency we like. Instead of overriding
 * `simulationPeriodic()` in your subsystems, you just use `ChurroSim.register`.
 */
public class ChurroSim {

  static ChurroSim instance;
  final List<ChurroSimEntity> m_entities;

  private ChurroSim() {
    m_entities = new ArrayList<>();
  }

  private static ChurroSim getInstance() {
    if (instance == null) {
      instance = new ChurroSim();
    }
    return instance;
  }

  private static List<ChurroSimEntity> getEntities() {
    return getInstance().m_entities;
  }

  public static void register(ChurroSimEntity entity) {
    getEntities().add(entity);
  }

  public static void iterate(double timeDeltaInSeconds) {
    for (ChurroSimEntity entity : getEntities()) {
      entity.iterate(timeDeltaInSeconds);
    }
  }
}
