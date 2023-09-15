package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;

@TeleOp(name = "NoTri", group = "Drivetrain")
public class asdasd extends OpMode {

    public DcMotor frontLWheel = null;
    public DcMotor frontRWheel = null;
    public DcMotor backLWheel = null;
    public DcMotor backRWheel = null;

    public double speedMod = 0.5;
    public boolean sprinting = false;


    @Override
    public void init() {
        frontLWheel = hardwareMap.dcMotor.get("frontLWheel");
        frontRWheel = hardwareMap.dcMotor.get("frontRWheel");
        backLWheel = hardwareMap.dcMotor.get("backLWheel");
        backRWheel = hardwareMap.dcMotor.get("backRWheel");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLWheel.setDirection(DcMotor.Direction.REVERSE);
        backRWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {


        telemetry.addData("RightY", gamepad1.right_stick_y);
        telemetry.addData("RightX", gamepad1.right_stick_x);
        telemetry.addData("RightTrigger", gamepad1.right_trigger);
        telemetry.addData("Speed Mod", speedMod);

        speedMod =  gamepad1.right_trigger - 0.5;
        if (speedMod < 0) {
            speedMod = 0.5;
        }

        if (gamepad1.right_stick_y != 0 && gamepad1.right_stick_x == 0) {
            frontLWheel.setPower(-gamepad1.right_stick_x * speedMod);
            frontRWheel.setPower(-gamepad1.right_stick_x * speedMod);
            backLWheel.setPower(gamepad1.right_stick_x * speedMod);
            backRWheel.setPower(gamepad1.right_stick_x * speedMod);
        } else {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            frontLWheel.setPower(v1 * 0.5);
            frontRWheel.setPower(v2 * 0.5);
            backLWheel.setPower(v3 * 0.5);
            backRWheel.setPower(v4 * 0.5);
        }

        telemetry.update();
    }
}