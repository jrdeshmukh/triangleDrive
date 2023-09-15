/*package org.firstinspires.ftc.teamcode.botconfigs;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MecDriveFlip;

import java.util.function.DoubleSupplier;

// mecanum drive bot with odometry for Pure Pursuit
public class rwer {

    // debugging device
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

    public rwer(Telemetry tele, HardwareMap map) {

        // store debugging device
        this.tele = tele;

        // initialize drive train
        motorFL = new Motor(map, "motorFL");
        motorFR = new Motor(map, "motorFR");
        motorBL = new Motor(map, "motorBL");
        motorBR = new Motor(map, "motorBR");
        drive = new MecDriveFlip(motorFL, motorFR, motorBL, motorBR);
    }
}
*/