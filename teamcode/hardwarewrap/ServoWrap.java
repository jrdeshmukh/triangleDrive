package org.firstinspires.ftc.teamcode.hardwarewrap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// base wrapper class for Servo device
public class ServoWrap {

    // telemtry device for debugging
    public Telemetry tele;

    // servo reference
    public Servo servo;

    // name in HardwareMap
    public String name;

    // outer limits for servo movement
    public double minLimit;
    public double maxLimit;

    // init, get servo from HardwareMap
    public ServoWrap(Telemetry tele, HardwareMap map, String name, double minLimit, double maxLimit) {

        // telemetry device for debugging
        this.tele = tele;

        // debugging data
        tele.addData("init", "servo: " + name);
        tele.update();

        // get motor reference
        servo = map.get(Servo.class, name);
        this.name = name;
        this.maxLimit = maxLimit;
        this.minLimit = minLimit;
    }

    // run to target position
    public void run(double target) {

        double thisTarget = (maxLimit - minLimit) * target + minLimit;

        tele.addData("serve set", thisTarget);

        // use linear interpolation to normalize a 0-1 range target to between the servo limits
        servo.setPosition(thisTarget);
    }
}
