package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivebase;
import org.firstinspires.ftc.teamcode.lib.teamlib.Utils;
import org.firstinspires.ftc.teamcode.lib.wpilib.Timer;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.kinematics.ChassisSpeeds;

@Photon
@Autonomous(preselectTeleOp = "DriveTest")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivebase drivebase = new Drivebase(this);

        telemetry.addData("Status", "Initialized");

        waitForStart();
        Timer timer = new Timer();
        timer.start();
        telemetry.addData("Status", "Running");
        while (opModeIsActive() && !timer.hasElapsed(5)) {
            double startTime = Utils.getTimeSeconds();

            drivebase.drive(new ChassisSpeeds(1, 0, 0));

            drivebase.periodic();

            telemetry.addData("Loop frequency", 1.0 / (Utils.getTimeSeconds() - startTime));
        }
    }
}
