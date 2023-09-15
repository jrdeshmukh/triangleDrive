package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import java.util.ArrayList;

// pure pursuit algorithm demo
//hi
@TeleOp(name="PursuitBotDemo3", group="PursuitBot")
public class PursuitBotDemo3 extends LinearOpMode {

    // robot reference
    public PursuitBot robot;

    // robot poses which form recorded path
    public ArrayList<Pose2d> recording;

    // movement parameters
    public double movementSpeed = 0.5;
    public double turnSpeed = 0.5;
    public double followRadius = 5;
    public double positionBuffer = 1;
    public double rotationBuffer = Math.toRadians(15);

    public boolean isAtHome = false;
    public boolean isDoneCorrectingRotation = false;
    public boolean positiveStartingX;
    public boolean positiveStartingY;

    @Override
    public void runOpMode() {

        // get reference to robot
        robot = new PursuitBot(telemetry, hardwareMap);

        // wait for user to start program
        waitForStart();

        // keep looping while program is running
        while (opModeIsActive()) {

            // loop through demo states
            RecordPath();
            ReturnHome();
            FollowPath();
            ReturnHome();
        }
    }

    // behaviour for robot driving and user path recording
    public void RecordPath() {

        // check that program is running
        if (opModeIsActive()) {

            // debug
            DebugPartial("record path");

            // reset recorded poses
            recording = new ArrayList<>();
            boolean recordInputLast = gamepad1.b;

            // keep looping while program is running and a is not pressed
            while (opModeIsActive() && (!gamepad1.a || recording.size() < 1)) {

                // debug
                DebugFull("record path");

                // update odometry system
                robot.odometry.update();

                // drive based on controller input
                robot.drive.driveRobotCentric(
                        -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                // add current pose to recording if b pressed
                boolean recordInputNew = gamepad1.b;
                if (recordInputNew && !recordInputLast) recording.add(robot.odometry.getPose());
                recordInputLast = recordInputNew;
            }

            // stop drive train
            robot.drive.stop();
        }
    }

    // follows recorded path with pure pursuit
    public void FollowPath() {

        // check that program is running
        if (opModeIsActive()) {

            // debug
            DebugPartial("follow path");

            // create start and end waypoints from current pose to last pose in recording
            Waypoint[] points = new Waypoint[recording.size() + 1];
            points[0] = new StartWaypoint(robot.odometry.getPose());
            points[points.length - 1] = new EndWaypoint(recording.get(recording.size() - 1),
                    movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

            // iterate through recorded poses and convert to waypoints
            for (int i = 0; i < recording.size() - 1; i++) {
                points[i + 1] = new GeneralWaypoint(recording.get(i),
                        movementSpeed, turnSpeed, followRadius);
            }

            // follow path formed by waypoints
            PurePursuitCommand command = new PurePursuitCommand(
                    robot.drive, robot.odometry, points);
            RunCommand(command, "follow path");
        }
    }

    // returns to origin position with pure pursuit
    public void ReturnHome() {

        // check that program is running
        if (opModeIsActive()) {

            // debug
            DebugPartial("return home");

            // create start and end waypoints from current pose to origin pose
            Waypoint start = new StartWaypoint(robot.odometry.getPose());
            Waypoint end = new EndWaypoint(new Pose2d(),
                    movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

            if(end.getPose().getY() <= 0) {positiveStartingY = false;} else {positiveStartingY = true;}
            if(end.getPose().getX() <= 0) {positiveStartingX = false;} else {positiveStartingX = true;}

            // follow path formed by waypoints
            PurePursuitCommand command = new PurePursuitCommand(
                    robot.drive, robot.odometry, start, end);
            RunCommand(command, "return home");

        }
    }

    // run command linearly
    public void RunCommand(PurePursuitCommand command, String state) {

        // follow path
        command.schedule();

        // loop while following
        while (opModeIsActive() && !command.isFinished()) {

            command.execute();
            robot.odometry.update();
            DebugFull(state);

            if(state.equals("return home"))
            {
                if(CheckForHome()) isAtHome = true;
                if(isAtHome)
                {
                    command.end(true);
                    RotationalCorrection();
                }
            }


            //check if drivetrain is behind the origin, then correct rotation if needed.

        }

        // end robot movement
        /*command.end(true);
        robot.drive.stop();*/

        // wait a second
        if (opModeIsActive()) sleep(1000);
    }

    public boolean CheckForHome()
    {
        double xPos = robot.odometry.getPose().getX();
        double yPos = robot.odometry.getPose().getY();
        boolean hasReachedHomeRange = false;

        if(!isAtHome)
        {
            if(positiveStartingY)
            {
                if(positiveStartingX)
                {
                    if (xPos <= 0) {if (yPos <= 0) hasReachedHomeRange = true;}
                }

                else
                {
                    if (xPos >= 0) {if (yPos <= 0) hasReachedHomeRange = true;}
                }
            }

            else
            {
                if(positiveStartingX)
                {
                    if (xPos <= 0) {if (yPos >= 0) hasReachedHomeRange = true;}
                }

                else
                {
                    if (xPos >= 0) {if (yPos >= 0) hasReachedHomeRange = true;}
                }
            }
        }

        if(hasReachedHomeRange)
        {
            return true;
        } else{
            return false;
        }
    }

    public void ManualOverride()
    {
        double xPos = robot.odometry.getPose().getX();
        double yPos = robot.odometry.getPose().getY();

        if(positiveStartingX && positiveStartingY)
        {
            while(xPos >= 0 && yPos >= 0)
            {
                robot.odometry.update();
                xPos = robot.odometry.getPose().getX();
                yPos = robot.odometry.getPose().getY();
                robot.drive.driveRobotCentric(-0.1, -0.1, 0.0);
            }
        }

        if(positiveStartingX && !positiveStartingY)
        {
            while(xPos >= 0 && yPos <= 0 && opModeIsActive())
            {
                robot.odometry.update();
                xPos = robot.odometry.getPose().getX();
                yPos = robot.odometry.getPose().getY();
                robot.drive.driveRobotCentric(-0.1, 0.1, 0.0);
            }
        }

        if(!positiveStartingX && positiveStartingY)
        {
            while(xPos <= 0 && yPos >= 0 && opModeIsActive())
            {
                robot.odometry.update();
                xPos = robot.odometry.getPose().getX();
                yPos = robot.odometry.getPose().getY();
                robot.drive.driveRobotCentric(0.1, -0.1, 0.0);
            }
        }

        if(!positiveStartingX && !positiveStartingY)
        {
            while(xPos <= 0 && yPos <= 0 && opModeIsActive())
            {
                robot.odometry.update();
                xPos = robot.odometry.getPose().getX();
                yPos = robot.odometry.getPose().getY();
                robot.drive.driveRobotCentric(0.1, 0.1, 0.0);
            }        }
    }

    public void RotationalCorrection()
    {
        if(isAtHome)
        {
            robot.drive.stop();

            if(robot.odometry.getPose().getRotation().getDegrees() != 0.0)
            {
                if(!isDoneCorrectingRotation) {

                    Rotation2d currentRotation = robot.odometry.getPose().getRotation();
//hello
                    if (currentRotation.getDegrees() >= 0.0) {
                        while (currentRotation.getDegrees() >= 0.0 && opModeIsActive()) {
                            robot.drive.driveRobotCentric(0.0, 0.0, -0.1);
                            robot.odometry.update();

                            if (currentRotation.getDegrees() <= 0) {
                                isDoneCorrectingRotation = true;
                                robot.drive.stop();
                                break;
                            }

                        }
                    }

                    else if (currentRotation.getDegrees() <= 0.0) {
                        while (currentRotation.getDegrees() <= 0.0 && opModeIsActive()) {
                            robot.drive.driveRobotCentric(0.0, 0.0, 0.1);
                            robot.odometry.update();

                            if (currentRotation.getDegrees() >= 0) {
                                isDoneCorrectingRotation = true;
                                robot.drive.stop();
                                break;
                            }
                        }
                    }

                    else
                    {
                        return;
                    }
                }

            }
        }
    }

    // debug program state with telemetry
    public void DebugPartial(String state) {

        telemetry.addData("state", state);
        telemetry.update();
    }

    // debug info on bot with telemetry
    public void DebugFull(String state) {

        telemetry.addData("state", state);
        telemetry.addData("point count", recording.size());
        telemetry.addData("current pose", robot.odometry.getPose());
        telemetry.addData("encoder vertical left", robot.encoderL.getAsDouble());
        telemetry.addData("encoder vertical right", robot.encoderR.getAsDouble());
        telemetry.addData("encoder horizontal", robot.encoderH.getAsDouble());
        telemetry.addData("input vertical", gamepad1.left_stick_x);
        telemetry.addData("input horizontal", -gamepad1.left_stick_y);
        telemetry.addData("input rotational", gamepad1.right_stick_x);
        telemetry.update();
    }
}
