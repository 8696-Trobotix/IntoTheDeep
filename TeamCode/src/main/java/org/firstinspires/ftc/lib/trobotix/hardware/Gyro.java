// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import java.util.function.Supplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Gyro {
  private final CachedAngles cache;

  public Gyro(OpMode opMode, String name, ImuOrientationOnRobot orientation) {
    var gyroInternal = opMode.hardwareMap.get(IMU.class, name);
    gyroInternal.initialize(new IMU.Parameters(orientation));

    cache = new CachedAngles(gyroInternal::getRobotYawPitchRollAngles);
  }

  public Rotation2d getYaw() {
    return new Rotation2d(cache.yawRad);
  }

  public Rotation2d getPitch() {
    return new Rotation2d(cache.pitchRad);
  }

  public Rotation2d getRoll() {
    return new Rotation2d(cache.rollRad);
  }

  private static class CachedAngles extends BaseOpMode.CachedValue {
    double yawRad;
    double pitchRad;
    double rollRad;

    final Supplier<YawPitchRollAngles> supplier;

    CachedAngles(Supplier<YawPitchRollAngles> supplier) {
      this.supplier = supplier;
    }

    @Override
    protected void update() {
      var axes = supplier.get();
      yawRad = Units.degreesToRadians(axes.getYaw());
      pitchRad = Units.degreesToRadians(axes.getPitch());
      rollRad = Units.degreesToRadians(axes.getRoll());
    }
  }
}
