package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GameController;
import org.firstinspires.ftc.teamcode.botconfigs.BBC;

import java.io.CharArrayWriter;


// mech drive train tele op for BBC
@TeleOp(name="BBCTeleadsd", group="FreightFrenzy")
public class BBCTele extends OpMode {

    // robot reference
    public BBC bot;

    // controller reference
    public GameController pad;


    //telemetry
    Telemetry tele;

    // init, get robot and controller
    @Override
    public void init() {

        bot = new BBC(tele, hardwareMap);
        pad = new GameController(new Gamepad[]{gamepad1, gamepad2});
    }

    // loop every frame
    @Override
    public void loop() {

        // update game controller input
        pad.update();
        // set drive train power with controller x, y, and rotational input
        bot.mechTrain.run(pad.doubleInputs[0][pad.stickLX], pad.doubleInputs[0][pad.stickLY], pad.doubleInputs[0][pad.stickRX]);
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
