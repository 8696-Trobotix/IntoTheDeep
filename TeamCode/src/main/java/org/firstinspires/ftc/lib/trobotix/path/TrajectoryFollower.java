// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.path;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.runOnce;

import org.firstinspires.ftc.lib.wpilib.Timer;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivebase;

public class TrajectoryFollower {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

  private final Drivebase drivebase;

  public TrajectoryFollower(Drivebase drivebase) {
    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    this.drivebase = drivebase;
  }

  public Command followPath(Trajectory trajectory) {
    Timer timer = new Timer();
    return runOnce(timer::start)
        .andThen(
            drivebase.run(
                () -> {
                  var sample = trajectory.sample(timer.get());
                  var pose = drivebase.getPose();

                  drivebase.fieldRelativeDrive(
                      new ChassisSpeeds(
                          sample.speeds().vxMetersPerSecond
                              + xController.calculate(pose.getX(), sample.pose().getX()),
                          sample.speeds().vyMetersPerSecond
                              + yController.calculate(pose.getY(), sample.pose().getY()),
                          sample.speeds().omegaRadiansPerSecond
                              + yawController.calculate(
                                  pose.getRotation().getRadians(),
                                  sample.pose().getRotation().getRadians())));
                }))
        .finallyDo(timer::stop);
  }
}
