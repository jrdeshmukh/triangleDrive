package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.botconfigs.MiniBot;

// simple two stick telemetry program
@TeleOp(name="MiniBotTele", group="MiniBots")
public class MiniBotTele extends OpMode {

    // robot reference
    public MiniBot robot;

    // called on robot initialization
    @Override
    public void init() {

        // get reference to robot
        robot = new MiniBot(telemetry, hardwareMap);
    }

    // looped after start
    @Override
    public void loop() {

        // move robot based on controller inputs
        robot.drive.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
    }
}
