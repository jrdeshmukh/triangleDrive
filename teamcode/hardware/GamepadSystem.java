package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class GamepadSystem {

    public GamepadEx gamepad1;
    public GamepadEx gamepad2;

    public GamepadSystem(OpMode opMode) {

        gamepad1 = new GamepadEx(opMode.gamepad1);
        gamepad2 = new GamepadEx(opMode.gamepad2);
    }
}
