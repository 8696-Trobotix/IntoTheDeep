// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.util.HashMap;
import org.firstinspires.ftc.lib.wpilib.Timer;

public class TelemetryThread {
  private static final Thread thread;

  static {
    thread = new Thread(TelemetryThread::run);
    thread.setDaemon(true);
    thread.setName("TelemetryThread");
  }

  public void start() {
    thread.start();
  }

  private static final HashMap<String, Object> PENDING_DATA = new HashMap<>();

  public static void addData(String name, Object object) {
    Utils.THREAD_LOCK.writeLock().lock();
    try {
      PENDING_DATA.put(name, object);
    } finally {
      Utils.THREAD_LOCK.writeLock().unlock();
    }
  }

  private static final double FREQUENCY = 10;

  private static void run() {
    double startTime = Utils.getTimeSeconds();
    while (true) {
      TelemetryPacket packet = new TelemetryPacket();
      Utils.THREAD_LOCK.readLock().lock();
      try {
        PENDING_DATA
            .entrySet()
            .iterator()
            .forEachRemaining((entry) -> packet.put(entry.getKey(), entry.getValue()));
        PENDING_DATA.clear();
      } finally {
        Utils.THREAD_LOCK.readLock().unlock();
      }
      double endTime = Utils.getTimeSeconds();
      double dT = endTime - startTime;
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
