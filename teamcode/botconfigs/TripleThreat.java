/*package org.firstinspires.ftc.teamcode.botconfigs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.hardware.GoofyTriangle;
import org.firstinspires.ftc.teamcode.hardwaresystems.MechDriveTrain;
import org.firstinspires.ftc.teamcode.hardwarewrap.DcMotorWrap;



// simple push bot with mechanum drive train
public class TripleThreat {

    // drive train properties
    public GoofyTriangle triangleTrain;
    public String[] driveTrainNames = {"rf", "rb", "lf", "lb"};
    public DcMotorWrap back = new DcMotorWrap(GoofyTriangle.tele, )
    //public String gyroName = "gyro";
    public double driveLinearSpeed = 1;
    public double driveTurnSpeed = 1;


    // init, get drive train
    public TripleThreat(Telemetry tele, HardwareMap map) {
        DcMotorWrap[] motors = new DcMotorWrap[4];
        for (int i = 0; i < motors.length; i++) {
            motors[i] = new DcMotorWrap(tele, map, driveTrainNames[i], 3, 1, 1, 1680);
        }

        //MechDriveTrain mechTrain;
        triangleTrain = new GoofyTriangle(tele, motors, driveLinearSpeed, driveTurnSpeed, 20);
        //driveTrain = new GyroOrientDriveTrain(mechTrain, gyro);
    }
}
*/