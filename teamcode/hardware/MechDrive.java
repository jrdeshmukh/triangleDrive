package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;



public class MechDrive {
    public Telemetry tele;
    public DcMotor frontLWheel;
    public DcMotor frontRWheel;
    public DcMotor backLWheel;
    public DcMotor backRWheel;
    public int blflip = 1;
    public int brflip = 1;
    public int flflip = 1;
    public int frflip = 1;
    public double speedMod = 1;
    public long timeRan;

    public MechDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, Telemetry tele2) {
        frontLWheel = fl;
        frontRWheel = fr;
        backLWheel = bl;
        backRWheel = br;
        tele = tele2;
    }


    public void flipMotor(DcMotor motor) {
        if (motor == frontLWheel) {
            flflip *= -1;
        } else if (motor == frontRWheel) {
            frflip *= -1;
        } else if (motor == backLWheel) {
             blflip *= -1;
        } else if (motor == backRWheel) {
            brflip *= -1;
        }

    }

    public void drive(float forward, float strafe, float turn, float right_trigger) {
        if (right_trigger != 0) {
            speedMod += right_trigger;
        }
        if (turn != 0) {
            frontLWheel.setPower(flflip * -turn * speedMod);
            frontRWheel.setPower(frflip * -turn * speedMod);
            backLWheel.setPower(blflip * turn * speedMod);
            backRWheel.setPower(brflip * turn * speedMod);
            speedMod = 1;
        } else {
            double r = Math.hypot(strafe, forward);
            double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
            double rightX = turn;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            //tele.addData("fl", v1*speedMod);
            //tele.addData("fr", v2*speedMod);
            //tele.addData("bl", v3*speedMod);
            //tele.addData("br", v4*speedMod);
            frontLWheel.setPower(v1 * speedMod);
            frontRWheel.setPower(v2 * speedMod);
            backLWheel.setPower(v3 * speedMod);
            backRWheel.setPower(v4 * speedMod);
            speedMod = 1;
            tele.update();
        }
    }
    public void driveAuto(float forward1, float strafe1, float turn1, float rightTrigger, double time) {
            double time2 = time* 1000;
            //tele.addData("f",time2);
            long time1 = System.currentTimeMillis();
            //tele.update();

            while (timeRan<=time2) {
                drive(forward1, strafe1, turn1, rightTrigger);
                timeRan = System.currentTimeMillis()-time1;
                //tele.addData("a", timeRan);
                //tele.update();

            }
            timeRan = 0;


        }
    }


