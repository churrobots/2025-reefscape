// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This singleton collects all simulation objects in a single place so that we
 * can update them on whatever frequency we like. Instead of overriding
 * `simulationPeriodic()` in your subsystems, you just use `ChurroSim.register`.
 */
public class HardwareRegistry {

  static HardwareRegistry instance;
  final List<TalonFX> m_talonFXList;
  final List<VictorSPX> m_victorSPXList;
  final List<SparkBase> m_sparkBaseList;
  final List<Pigeon2> m_pigeon2List;

  private HardwareRegistry() {
    m_talonFXList = new ArrayList<>();
    m_victorSPXList = new ArrayList<>();
    m_sparkBaseList = new ArrayList<>();
    m_pigeon2List = new ArrayList<>();
  }

  private static HardwareRegistry getInstance() {
    if (instance == null) {
      instance = new HardwareRegistry();
    }
    return instance;
  }

  public static void registerHardware(Object device) {
    if (device instanceof TalonFX) {
      registerHardware((TalonFX) device);
    } else if (device instanceof VictorSPX) {
      registerHardware((VictorSPX) device);
    } else if (device instanceof SparkBase) {
      registerHardware((SparkBase) device);
    } else if (device instanceof Pigeon2) {
      registerHardware((Pigeon2) device);
    } else {
      throw new IllegalArgumentException("Unknown device type: " + device.getClass().getName());
    }
  }

  public static void registerHardware(TalonFX device) {
    getInstance().m_talonFXList.add(device);
  }

  public static void registerHardware(VictorSPX device) {
    getInstance().m_victorSPXList.add(device);
  }

  public static void registerHardware(SparkBase device) {
    getInstance().m_sparkBaseList.add(device);
  }

  public static void registerHardware(Pigeon2 device) {
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

  public static SparkBase getSparkBase(int canId) {
    for (SparkBase device : getInstance().m_sparkBaseList) {
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

  public static void dumpDeviceFaultsToNetworkTables() {
    // TODO: implement this for other devices? like VictorSPX's?
    for (Pigeon2 device : getInstance().m_pigeon2List) {
      boolean faultHardware = device.getFault_Hardware().getValue() == true;
      boolean faultUndervoltage = device.getFault_Undervoltage().getValue() == true;
      boolean faultBootDuringEnable = device.getFault_BootDuringEnable().getValue() == true;
      boolean faultUnlicensedFeatureInUse = device.getFault_UnlicensedFeatureInUse().getValue() == true;
      boolean faultBootupAccelerometer = device.getFault_BootupAccelerometer().getValue() == true;
      boolean faultBootupGyroscope = device.getFault_BootupGyroscope().getValue() == true;
      boolean faultBootupMagnetometer = device.getFault_BootupMagnetometer().getValue() == true;
      boolean faultBootIntoMotion = device.getFault_BootIntoMotion().getValue() == true;
      boolean faultDataAcquiredLate = device.getFault_DataAcquiredLate().getValue() == true;
      boolean faultLoopTimeSlow = device.getFault_LoopTimeSlow().getValue() == true;
      boolean faultSaturatedMagnetometer = device.getFault_SaturatedMagnetometer().getValue() == true;
      boolean faultSaturatedAccelerometer = device.getFault_SaturatedAccelerometer().getValue() == true;
      boolean faultSaturatedGyroscope = device.getFault_SaturatedGyroscope().getValue() == true;
      boolean hasAnyFaults = faultHardware || faultUndervoltage || faultBootDuringEnable
          || faultUnlicensedFeatureInUse || faultBootupAccelerometer || faultBootupGyroscope
          || faultBootupMagnetometer || faultBootIntoMotion || faultDataAcquiredLate || faultLoopTimeSlow
          || faultSaturatedMagnetometer || faultSaturatedAccelerometer || faultSaturatedGyroscope;
      boolean isGood = device.isConnected() && !hasAnyFaults;
      SmartDashboard.putBoolean("HardwareRegistry/Faults/Pigeon2[" + device.getDeviceID() + "]",
          isGood);
    }
    for (SparkBase device : getInstance().m_sparkBaseList) {
      boolean isGood = !device.hasActiveFault() && !device.hasActiveWarning();
      // TODO: check for supply voltage dropping out? how would we detect the white
      // wires being disconnected?
      SmartDashboard.putBoolean("HardwareRegistry/Faults/SparkBase[" + device.getDeviceId() + "]",
          isGood);
    }
    for (TalonFX device : getInstance().m_talonFXList) {
      boolean faultHardware = device.getFault_Hardware().getValue() == true;
      boolean faultProcTemp = device.getFault_ProcTemp().getValue() == true;
      boolean faultDeviceTemp = device.getFault_DeviceTemp().getValue() == true;
      boolean faultUndervoltage = device.getFault_Undervoltage().getValue() == true;
      boolean faultBootDuringEnable = device.getFault_BootDuringEnable().getValue() == true;
      boolean faultUnlicensedFeatureInUse = device.getFault_UnlicensedFeatureInUse().getValue() == true;
      boolean faultBridgeBrownout = device.getFault_BridgeBrownout().getValue() == true;
      boolean faultRemoteSensorReset = device.getFault_RemoteSensorReset().getValue() == true;
      boolean faultMissingDifferentialFX = device.getFault_MissingDifferentialFX().getValue() == true;
      boolean faultRemoteSensorPosOverflow = device.getFault_RemoteSensorPosOverflow().getValue() == true;
      boolean faultOverSupplyV = device.getFault_OverSupplyV().getValue() == true;
      boolean faultUnstableSupplyV = device.getFault_UnstableSupplyV().getValue() == true;
      boolean faultReverseHardLimit = device.getFault_ReverseHardLimit().getValue() == true;
      boolean faultForwardHardLimit = device.getFault_ForwardHardLimit().getValue() == true;
      boolean faultReverseSoftLimit = device.getFault_ReverseSoftLimit().getValue() == true;
      boolean faultForwardSoftLimit = device.getFault_ForwardSoftLimit().getValue() == true;
      boolean faultMissingSoftLimitRemote = device.getFault_MissingSoftLimitRemote().getValue() == true;
      boolean faultMissingHardLimitRemote = device.getFault_MissingHardLimitRemote().getValue() == true;
      boolean faultRemoteSensorDataInvalid = device.getFault_RemoteSensorDataInvalid().getValue() == true;
      boolean faultFusedSensorOutOfSync = device.getFault_FusedSensorOutOfSync().getValue() == true;
      boolean faultStatorCurrLimit = device.getFault_StatorCurrLimit().getValue() == true;
      boolean faultSupplyCurrLimit = device.getFault_SupplyCurrLimit().getValue() == true;
      boolean faultUsingFusedCANcoderWhileUnlicensed = device.getFault_UsingFusedCANcoderWhileUnlicensed()
          .getValue() == true;
      boolean faultStaticBrakeDisabled = device.getFault_StaticBrakeDisabled().getValue() == true;
      boolean hasAnyFaults = faultHardware || faultProcTemp || faultDeviceTemp || faultUndervoltage
          || faultBootDuringEnable || faultUnlicensedFeatureInUse || faultBridgeBrownout || faultRemoteSensorReset
          || faultMissingDifferentialFX || faultRemoteSensorPosOverflow || faultOverSupplyV || faultUnstableSupplyV
          || faultReverseHardLimit || faultForwardHardLimit || faultReverseSoftLimit || faultForwardSoftLimit
          || faultMissingSoftLimitRemote || faultMissingHardLimitRemote || faultRemoteSensorDataInvalid
          || faultFusedSensorOutOfSync || faultStatorCurrLimit || faultSupplyCurrLimit
          || faultUsingFusedCANcoderWhileUnlicensed || faultStaticBrakeDisabled;
      boolean isDisconnected = !device.isConnected();
      boolean isGood = !hasAnyFaults && !isDisconnected;
      SmartDashboard.putBoolean("HardwareRegistry/Faults/TalonFX[" + device.getDeviceID() + "]",
          isGood);
    }
  }
}
