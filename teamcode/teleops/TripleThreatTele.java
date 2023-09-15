package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GameController;
import org.firstinspires.ftc.teamcode.hardware.GoofyTriangle;
import org.firstinspires.ftc.teamcode.hardwarewrap.DcMotorWrap;


// mech drive train tele op for BBC
@TeleOp(name="TripleThreatTele", group="Centerstae")
public class TripleThreatTele extends OpMode {

    // robot reference
    public GoofyTriangle bot;

    // controller reference
    public GameController pad;
    Telemetry tele;
    public DcMotor back = hardwareMap.dcMotor.get("back");
    public DcMotor frontL = hardwareMap.dcMotor.get("frontL");
    public DcMotor frontR = hardwareMap.dcMotor.get("frontR");



    //telemetry


    // init, get robot and controller
    @Override
    public void init() {

        bot = new GoofyTriangle( back, frontL, frontR, tele);
        pad = new GameController(new Gamepad[]{gamepad1, gamepad2});
    }


    // loop every frame
    @Override
    public void loop() {
        // update game controller input
        pad.update();
        tele.addData("Gamepad Stick LX:", gamepad1.left_stick_x);
        tele.addData("Gamepad Stick LY:", gamepad1.left_stick_y);
        tele.addData("Gamepad Stick RX:", gamepad1.right_stick_x);
        tele.addData("Gamepad Right Trigger:", gamepad1.right_trigger);
        // set drive train power with controller x, y, and rotational input
        bot.drive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x, gamepad1.right_trigger);
    }
}
/*
@TeleOp (name="", group="")
public class BBCTele extends OpMode {

    public BBCbot;
// initialize hardware devices (additional motors/servos if needed)
    motorLeft = hardwareMap.dcMotor.get("motorLeft");
    public GameController pad;
    public void init() {
        robot = new BBCBot(telemetry, hardwareMap);

    }

    public void loop() {
        motorLeft.setPower(-gamepad1.left_stick_y);
        motorRight.setPower(-gamepad1.right_stick_y);
    }
}
 */
