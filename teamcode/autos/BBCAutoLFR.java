package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.MechDrive;

;


@Autonomous(name = "BBCAutoLFR", group = "Drivetrain")
public class BBCAutoLFR extends LinearOpMode {

    public Telemetry tele;
    public MechDrive robot;
    public Claw claw;
    public DcMotor frontLWheel = null;
    public DcMotor frontRWheel = null;
    public DcMotor backLWheel = null;
    public DcMotor backRWheel = null;
    public CRServo Claw;
    public DcMotor up;
    public double speedMod = 0.5;
    public boolean sprinting = false;




    public void runOpMode() {
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

        double g2ry = gamepad2.right_stick_y;
        telemetry.addData("RightY", gamepad1.right_stick_y);
        telemetry.addData("RightX", gamepad1.right_stick_x);
        telemetry.addData("RightTrigger", gamepad1.right_trigger);
        telemetry.addData("Speed Mod", speedMod);
        telemetry.addData("MoveUp", g2ry);
        claw.moveUp(g2ry);
        telemetry.addData("running", "running");
        waitForStart();
        robot.driveAuto(0,1,0,0,0.75);
        robot.driveAuto(1, 0, 0, 0, 2.75);

        robot.driveAuto(0,-1,0,0,1.75);
        robot.driveAuto(1, 0, 0, 0, 0.75);


        telemetry.update();


    }

}
