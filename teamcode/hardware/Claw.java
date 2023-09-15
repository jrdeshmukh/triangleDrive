package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Claw {
    public CRServo Claw;
    public DcMotor upClaw;
    public Telemetry tele;

    public Claw(CRServo claw, DcMotor up, Telemetry Tele) {
        Claw = claw;
        upClaw = up;
        tele = Tele;
    }

    public void move(float power) {
        if (power != 0) {
            tele.addData("Power", power);
            tele.update();
            Claw.setPower(power);

        }
        else {
            Claw.setPower(0);
        }
    }

    public void moveUp(double moveup) {
        upClaw.setPower(moveup);


    }
}