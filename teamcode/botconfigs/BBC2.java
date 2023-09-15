/*package org.firstinspires.ftc.teamcode.botconfigs;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MecDriveFlip;

import java.util.function.DoubleSupplier;

public class BBC2 {
    public Telemetry tele;

    // mecanum wheel drive train
    public MecanumDrive drive;
    public Motor motorFL;
    public Motor motorFR;
    public Motor motorBL;
    public Motor motorBR;


    // hardware specifications
    public double wheelDiameter = 2.5;
    public double wheelCircumference = wheelDiameter * Math.PI;
    public BBC2(Telemetry tele, HardwareMap map) {

        // store debugging device
        this.tele = tele;

        // initialize drive train
        motorFL = new Motor(map, "lf");
        motorFR = new Motor(map, "rf");
        motorBL = new Motor(map, "lb");
        motorBR = new Motor(map, "rb");
        drive = new MecDriveFlip(motorFL, motorFR, motorBL, motorBR);

    }
}
*/
