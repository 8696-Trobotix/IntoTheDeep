// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

public class EndableThread extends Thread {
  public final String NAME;
  private volatile boolean ENABLED = true;

  public EndableThread(String name) {
    this.NAME = name;
    Utils.registerThread(this);
    setDaemon(true);
  }

  @Override
  public void run() {
    preStart();
    while (ENABLED) {
      loop();
    }
    end();
  }

  /** Runs at the start of the thread. */
  public void preStart() {}

  /** Repeats until the op mode is stopped. */
  public void loop() {}

  /** Runs when the op mode is stopped. */
  public void end() {}

  protected void disable() {
    ENABLED = false;
  }
}
