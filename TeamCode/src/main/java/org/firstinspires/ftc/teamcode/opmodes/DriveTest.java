package org.firstinspires.ftc.teamcode.opmodes;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivebase;

@Photon
@TeleOp
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivebase drivebase = new Drivebase(this);

        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            drivebase.teleopDrive(-this.gamepad1.left_stick_x, -this.gamepad1.left_stick_y, -this.gamepad1.right_stick_x);
        }
    }
}
