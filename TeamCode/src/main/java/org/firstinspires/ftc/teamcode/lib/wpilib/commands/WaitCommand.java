package org.firstinspires.ftc.teamcode.lib.wpilib.commands;

import org.firstinspires.ftc.teamcode.lib.wpilib.Timer;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class WaitCommand extends Command {
  /** The timer used for waiting. */
  protected Timer m_timer = new Timer();

  private final double m_duration;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  @SuppressWarnings("this-escape")
  public WaitCommand(double seconds) {
    m_duration = seconds;
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }
}
