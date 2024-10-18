// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import java.util.ArrayList;

/**
 * A {@link Thread} that runs a method on a loop, that is able to end cleanly when an op mode ends.
 */
public abstract class EndableThread extends Thread {
  private static volatile boolean ENABLED = true;

  private final String NAME;

  public EndableThread(String name) {
    this.NAME = name;
    threads.add(this);
    setDaemon(true);
  }

  @Override
  public final void run() {
    preStart();
    double lastTime = Utils.getTimeSeconds();
    while (ENABLED) {
      loop();
      double currentTime = Utils.getTimeSeconds();
      Telemetry.addTiming(NAME, currentTime - lastTime);
      lastTime = currentTime;
    }
    end();
  }

  /** Runs at the start of the thread. */
  public void preStart() {}

  /** Repeats until the op mode is stopped. */
  public void loop() {}

  /** Runs when the op mode is stopped. */
  public void end() {}

  private static final ArrayList<EndableThread> threads = new ArrayList<>();

  protected static void startThreads() {
    ENABLED = true;
    threads.forEach(
        (thread) -> {
          try {
            thread.start();
          } catch (IllegalThreadStateException e) {
            throw new RuntimeException(
                "Failed to start " + thread.NAME + " due to an IllegalThreadStateException: " + e);
          }
        });
  }

  protected static void endThreads() {
    ENABLED = false;
    threads.clear();
  }
}
