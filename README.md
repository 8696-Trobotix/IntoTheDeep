# Team 8696 2024 Team Code

The repository for Trobotix's code for the 2024 FTC Season, Into The Deep.

## Structure

### [`org.firstinspires.ftc.lib`](https://github.com/8696-Trobotix/IntoTheDeep/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/lib)

Library classes.

The FTC SDK is extremely barebones, slow without the use of Photon, generally infuriating, and the
standard third-party tooling sucks. The FRC software ecosystem, on the other hand, is mature, broad,
performant, makes sense, and most of what you need is conveniently bundled into the standard library
of WPILib. This package ports FRC tooling and practices to FTC, making programming easier.

- [`org.firstinspires.ftc.lib.wpilib`](https://github.com/8696-Trobotix/IntoTheDeep/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/lib/wpilib)
    - A port of WPILib's Java classes to the FTC SDK.
- [`org.firstinspires.ftc.lib.trobotix`](https://github.com/8696-Trobotix/IntoTheDeep/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/lib/trobotix)
    - Library for various tools not covered by the above.

### [`org.firstinspires.ftc.teamcode`](https://github.com/8696-Trobotix/IntoTheDeep/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode)

The main code for the robot.

- [`org.firstinspires.ftc.teamcode.hardware`](https://github.com/8696-Trobotix/IntoTheDeep/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware)
  - Package for hardware control code.
  - [`org.firstinspires.ftc.teamcode.hardware.drive`](https://github.com/8696-Trobotix/IntoTheDeep/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/drive)
    - Code for the drivebase.
- [`org.firstinspires.ftc.teamcode.opmodes`](https://github.com/8696-Trobotix/IntoTheDeep/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes)
  - Package for op modes that the robot runs.
  - [`org.firstinspires.ftc.teamcode.opmodes.teleop`](https://github.com/8696-Trobotix/IntoTheDeep/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop)
    - Teleop modes.
  - [`org.firstinspires.ftc.teamcode.opmodes.auto`](https://github.com/8696-Trobotix/IntoTheDeep/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto)
    - Auto modes.
