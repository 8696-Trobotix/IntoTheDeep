// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.util.HashMap;
import org.firstinspires.ftc.lib.wpilib.Timer;

public class DashboardTelemetry {
  private static final boolean ENABLED = true;

  private static TelemetryThread instance;

  private static TelemetryThread getThread() {
    if (ENABLED && instance == null) {
      instance = new TelemetryThread();
      instance.start();
    }
    return instance;
  }

  public static void addData(String name, Object data) {
    if (!ENABLED) {
      return;
    }
    getThread().addData(name, data);
  }

  public static void addTiming(String name, double timingSeconds) {
    if (!ENABLED) {
      return;
    }
    getThread().addTiming(name, timingSeconds);
  }

  private static class TelemetryThread extends Thread {
    TelemetryThread() {
      setName("Telemetry Thread");
      setDaemon(true);
    }

    private final HashMap<String, Object> pendingData = new HashMap<>();

    @Override
    public void run() {
      //noinspection InfiniteLoopStatement
      while (true) {
        TelemetryPacket packet = new TelemetryPacket();
        synchronized (pendingData) {
          pendingData
              .entrySet()
              .iterator()
              .forEachRemaining((entry) -> packet.put(entry.getKey(), entry.getValue()));
          pendingData.clear();
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        Timer.delay(.1);
      }
    }

    public void addData(String name, Object data) {
      synchronized (pendingData) {
        pendingData.put(name, data);
      }
    }

    public void addTiming(String name, double timingSeconds) {
      synchronized (pendingData) {
        pendingData.put(name + "/Timing (ms)", timingSeconds * 1000);
        pendingData.put(name + "/Frequency (hz)", 1.0 / timingSeconds);
      }
    }
  }
}
