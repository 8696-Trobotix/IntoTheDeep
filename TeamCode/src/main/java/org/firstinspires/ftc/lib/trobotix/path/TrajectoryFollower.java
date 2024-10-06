// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.path;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.run;
import static org.firstinspires.ftc.lib.wpilib.commands.Commands.runOnce;

import java.util.function.Consumer;
import java.util.function.Supplier;
import org.firstinspires.ftc.lib.wpilib.Timer;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;

public class TrajectoryFollower {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<ChassisSpeeds> driveMethod;

  public TrajectoryFollower(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> driveMethod) {
    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    this.poseSupplier = poseSupplier;
    this.driveMethod = driveMethod;
  }

  public Command followPath(Trajectory trajectory) {
    Timer timer = new Timer();
    return runOnce(timer::start)
        .andThen(
            run(
                () -> {
                  Trajectory.TrajectorySample sample = trajectory.sample(timer.get());
                  Pose2d pose = poseSupplier.get();

                  driveMethod.accept(
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
