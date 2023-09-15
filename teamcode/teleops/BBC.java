package org.firstinspires.ftc.teamcode.teleops;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MechDrive;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name = "BBCTele", group = "Drivetrain")
public class BBC extends OpMode {
    public float speed = (float) 0.75000123;
    public Telemetry tele;
    public MechDrive robot;
    public Claw claw;
    public DcMotor frontLWheel = null;
    public DcMotor frontRWheel = null;
    public DcMotor backLWheel = null;
    public DcMotor backRWheel = null;
    public CRServo Claw;
    public DcMotor up;
    public float turnSpeed = (float) 0.6;
    public double speedMod = 0.5;
    public boolean sprinting = false;


    @Override
    public void init() {
        Claw = hardwareMap.crservo.get("Claw");
        frontLWheel = hardwareMap.dcMotor.get("frontLWheel");
        frontRWheel = hardwareMap.dcMotor.get("frontRWheel");
        backLWheel = hardwareMap.dcMotor.get("backLWheel");
        backRWheel = hardwareMap.dcMotor.get("backRWheel");
        up = hardwareMap.dcMotor.get("up");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLWheel.setDirection(DcMotor.Direction.REVERSE);
        backRWheel.setDirection(DcMotor.Direction.REVERSE);
        robot = new MechDrive(frontLWheel, frontRWheel, backLWheel, backRWheel, telemetry);
        claw = new Claw(Claw, up, telemetry);
    }

    @Override
    public void loop() {
        double g2ry = gamepad2.right_stick_y;
        telemetry.addData("RightY", gamepad1.right_stick_y);
        telemetry.addData("RightX", gamepad1.right_stick_x);
        telemetry.addData("RightTrigger", gamepad1.right_trigger);
        telemetry.addData("Speed Mod", speedMod);
        telemetry.addData("MoveUp", g2ry);
        if (gamepad2.left_trigger != 0) {
            claw.moveUp(g2ry);
        }
        else {
            claw.moveUp(0.75*g2ry);
        }
        if (gamepad2.right_trigger != 0) {
            claw.moveUp(0.5*g2ry);
        }
        else {
            claw.moveUp(0.75*g2ry);
        }

            telemetry.addData("running", "running");
        telemetry.addData("lefttrigger", gamepad1.left_trigger);
        if (gamepad1.left_trigger == 0) {
            robot.drive(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed,
                    gamepad1.right_stick_x * turnSpeed,
                    gamepad1.right_trigger * speed
            );
        }
        else if (gamepad1.left_trigger != 0) {
            robot.drive(
                    -gamepad1.left_stick_y * (float) 0.5,
                    -gamepad1.left_stick_x * (float) 0.5,
                    gamepad1.right_stick_x * turnSpeed *  (float) 0.5,
                    gamepad1.right_trigger *  (float) 0.5
            );
        }
        claw.move(gamepad2.left_stick_x);
        
        telemetry.update();
    }
}