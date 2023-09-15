package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.botconfigs.BBC2;
import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;


@TeleOp(name="BBCTele2", group="ClawLiftBot")
public class BBCTele2 extends OpMode {

    BBC2 robot;



    public double turnSpeed = 0.5;
    public double linearSpeed = 1;

    // input system reference
    GamepadSystem input;

    // called on program initialization
    @Override
    public void init() {

        robot = new BBC2(telemetry, hardwareMap);
        telemetry.addData("init","works");

        // initialize hardware devices
        input = new GamepadSystem(this);
        telemetry.addData("gamepad","works");
    }

    // called repeatedly during program
    @Override
    public void loop() {
        telemetry.addData("fb",input.gamepad1.getLeftX());
        telemetry.addData("lr",input.gamepad1.getLeftY());
        telemetry.addData("turn",input.gamepad1.getRightX());

        robot.drive.driveRobotCentric(
                input.gamepad1.getRightX() * linearSpeed,
                input.gamepad1.getLeftX() * linearSpeed,
                input.gamepad1.getLeftY() * turnSpeed);


    }
}
