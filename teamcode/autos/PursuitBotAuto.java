package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;

// pure pursuit algorithm demo
@Autonomous(name="PursuitBotAuto", group="PursuitBot")
public class PursuitBotAuto extends LinearOpMode {

    // robot reference
    public PursuitBot robot;

    // movement parameters
    public double movementSpeed = 0.925;
    public double turnSpeed = 0.925;
    public double followRadius = 7.5;
    public double positionBuffer = 0.25;
    public double rotationBuffer = Math.toRadians(0.2);

    @Override
    public void runOpMode() {

        // get reference to robot
        robot = new PursuitBot(telemetry, hardwareMap);

        // wait for user to start program
        waitForStart();

        // declare waypoints
        Waypoint start = new StartWaypoint(new Pose2d());
        Waypoint middle = new GeneralWaypoint(12, 24,
                movementSpeed, turnSpeed, followRadius);
        Waypoint end = new EndWaypoint(new Pose2d(),
                movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

        // form path
        Path path = new Path(start, middle, end);
        path.init();

        // make bot follow path
        //path.followPath(robot.drive, robot.odometry);
    }
}
