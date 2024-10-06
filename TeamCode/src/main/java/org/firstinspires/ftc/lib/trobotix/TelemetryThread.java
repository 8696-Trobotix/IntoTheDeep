// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.util.HashMap;
import org.firstinspires.ftc.lib.wpilib.Timer;

public class TelemetryThread {
  private static final Thread thread;

  private static final boolean ENABLED = true;

  static {
    thread = new Thread(TelemetryThread::run);
    thread.setDaemon(true);
    thread.setName("TelemetryThread");
  }

  public static void start() {
    if (ENABLED) {
      thread.start();
    }
  }

  private static final HashMap<String, Object> PENDING_DATA = new HashMap<>();

  public static void addData(String name, Object object) {
    if (!ENABLED) {
      return;
    }
    synchronized (PENDING_DATA) {
      PENDING_DATA.put(name, object);
    }
  }

  public static void addTiming(String name, double timingSeconds) {
    if (!ENABLED) {
      return;
    }
    synchronized (PENDING_DATA) {
      PENDING_DATA.put(name + "/Timing (ms)", timingSeconds * 1000);
      PENDING_DATA.put(name + "Frequency (hz)", 1.0 / timingSeconds);
    }
  }

  private static final double FREQUENCY = 10;

  private static void run() {
    FtcDashboard.getInstance().setTelemetryTransmissionInterval((int) Math.round(1000 / FREQUENCY));
    var startTime = Utils.getTimeSeconds();
    while (Utils.opModeActiveSupplier.getAsBoolean()) {
      var packet = new TelemetryPacket();

      synchronized (PENDING_DATA) {
        PENDING_DATA
            .entrySet()
            .iterator()
            .forEachRemaining((entry) -> packet.put(entry.getKey(), entry.getValue()));
        PENDING_DATA.clear();
      }

      var endTime = Utils.getTimeSeconds();
      var dT = endTime - startTime;
      packet.put("TelemetryThread/Timing (ms)", dT * 1000);
      packet.put("TelemetryThread/Runtime %", FREQUENCY * dT * 100);

      FtcDashboard.getInstance().sendTelemetryPacket(packet);

      startTime = endTime;

      if (dT < 1.0 / FREQUENCY) {
        Timer.delay((1.0 / FREQUENCY) - dT);
      }
    }
  }
}
