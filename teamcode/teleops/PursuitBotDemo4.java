package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.Convert;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import java.util.ArrayList;

// pure pursuit algorithm demo
//hi
@TeleOp(name="PursuitBotDemo4", group="PursuitBot")
public class PursuitBotDemo4 extends LinearOpMode {

    // robot reference
    public PursuitBot robot;

    // robot poses which form recorded path
    public ArrayList<Pose2d> recording;
    public ArrayList<Translation2d> returnHomeInteriorWaypoints = new ArrayList<Translation2d>();
    public ArrayList<Translation2d> followPathInteriorWaypoints = new ArrayList<Translation2d>();

    // movement parameters
    public double movementSpeed = 0.5;
    public double turnSpeed = 0.5;
    public double followRadius = 5;
    public double positionBuffer = 1;
    public double rotationBuffer = Math.toRadians(15);
    public double maxVelocity = 0.5;
    public double maxAcceleration = 0.5;

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

    public Translation2d ConvertToTranslation2d(Pose2d pose)
    {
        double xValue = pose.getX();
        double yValue = pose.getY();

        return new Translation2d(xValue, yValue);
    }

    // follows recorded path with pure pursuit
    public void FollowPath() {

        // check that program is running
        if (opModeIsActive()) {

            // debug
            DebugPartial("follow path");

            // create start and end waypoints from current pose to last pose in recording
            Pose2d start = robot.odometry.getPose();
            Pose2d end = recording.get(recording.size() - 1);

            Waypoint[] waypoints = new Waypoint[recording.size() + 3];
            waypoints[0] = new StartWaypoint(start);
            waypoints[recording.size() - 1] = new EndWaypoint(end,
                    movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

            for (int i = 1; i < recording.size() - 2; i++)
            {
                double xPos = waypoints[i].getPose().getX();
                double yPos = waypoints[i].getPose().getY();
                Translation2d translation = new Translation2d(xPos, yPos);
                followPathInteriorWaypoints.add(translation);
            }

            TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
            Trajectory returnHomeTrajectory = TrajectoryGenerator.generateTrajectory(
                    start, followPathInteriorWaypoints, end, config);

            double seconds = returnHomeTrajectory.getTotalTimeSeconds();

            for (int i = 0; i < waypoints.length - 1; i++)
            {
                waypoints[i + 1] = new PointTurnWaypoint(
                        returnHomeTrajectory.sample((1.0/(i + 1.0) * seconds)).poseMeters,
                        movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);
            }

            waypoints[0] = new StartWaypoint(start);
            waypoints[waypoints.length - 1] = new EndWaypoint(end,
                    movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

            // follow path formed by waypoints
            PurePursuitCommand command = new PurePursuitCommand(
                    robot.drive, robot.odometry, waypoints);
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
            Pose2d start = robot.odometry.getPose();
            Pose2d end = new Pose2d();

            returnHomeInteriorWaypoints.add(new Translation2d(0, 0));

            //positiveStartingY = (start.getY() >= 0);

            TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
            Trajectory returnHomeTrajectory = TrajectoryGenerator.generateTrajectory(
                    start, returnHomeInteriorWaypoints, end, config);

            // follow path formed by waypoints
            double seconds = returnHomeTrajectory.getTotalTimeSeconds();
            Waypoint[] waypoints = new Waypoint[102];
            waypoints[0] = new StartWaypoint(start);
            waypoints[waypoints.length - 1] = new EndWaypoint(end, movementSpeed,
                    turnSpeed, followRadius, positionBuffer, rotationBuffer);

            for (int i = 0; i < 100; i++)
            {
                waypoints[i + 1] = new GeneralWaypoint(
                        returnHomeTrajectory.sample(((i / 100.0) * seconds)).poseMeters,
                        movementSpeed, turnSpeed, followRadius);
            }

            PurePursuitCommand command = new PurePursuitCommand(
                    robot.drive, robot.odometry, waypoints);
            RunCommand(command, "return home");

        }
    }

    // run command linearly
    public void RunCommand(PurePursuitCommand command, String state) {

        if(state.equals("return home"))
        {
            double currentRotation = robot.odometry.getPose().getRotation().getDegrees();

            if(currentRotation <= 90 && currentRotation >= -90 && robot.odometry.getPose().getY() >= 0)
            {
                RotationalCorrection();
                robot.drive.stop();
            }
        }

        // follow path
        command.schedule();

        // loop while following
        while (opModeIsActive() && !command.isFinished()) {

            command.execute();
            robot.odometry.update();
            DebugFull(state);
        }

        command.end(true);
        robot.drive.stop();

        if(state.equals("return home"))
        {
            HomeRotationalCorrection();
        }

        // wait a second
        if (opModeIsActive()) sleep(1000);
    }

    public void RotationalCorrection()
    {
        robot.drive.stop();

        if(robot.odometry.getPose().getRotation().getDegrees() != 179.9)
        {
            if(!isDoneCorrectingRotation) {

                Rotation2d currentRotation = robot.odometry.getPose().getRotation();

                if (currentRotation.getDegrees() <= 179.9 && currentRotation.getDegrees() >= 0) {
                    while ((currentRotation.getDegrees() <= 179.0) && opModeIsActive()) {
                        robot.drive.driveRobotCentric(0.0, 0.0, 2.0);
                        robot.odometry.update();
                        currentRotation = robot.odometry.getPose().getRotation();
                        DebugPartial("correcting rotation: " + currentRotation);

                        if (currentRotation.getDegrees() >= 179.0) {
                            isDoneCorrectingRotation = true;
                            robot.drive.stop();
                            break;
                        }

                    }
                }

                else if (currentRotation.getDegrees() >= -179.9 && currentRotation.getDegrees() < 0) {
                    while (currentRotation.getDegrees() >= -179.0 && opModeIsActive()) {
                        robot.drive.driveRobotCentric(0.0, 0.0, -2.0);
                        robot.odometry.update();
                        currentRotation = robot.odometry.getPose().getRotation();
                        DebugPartial("correcting rotation: " + currentRotation);

                        if (currentRotation.getDegrees() <= 179.0) {
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

    public void HomeRotationalCorrection()
    {
        robot.drive.stop();

        if(robot.odometry.getPose().getRotation().getDegrees() != 0.0)
        {
            if(!isDoneCorrectingRotation) {

                Rotation2d currentRotation = robot.odometry.getPose().getRotation();
//hello
                if (currentRotation.getDegrees() >= 0.0) {
                    while (currentRotation.getDegrees() >= 0.0 && opModeIsActive()) {
                        robot.drive.driveRobotCentric(0.0, 0.0, -2.0);
                        robot.odometry.update();
                        currentRotation = robot.odometry.getPose().getRotation();
                        DebugPartial("correcting rotation: " + currentRotation);

                        if (currentRotation.getDegrees() <= 0) {
                            isDoneCorrectingRotation = true;
                            robot.drive.stop();
                            break;
                        }

                    }
                }

                else if (currentRotation.getDegrees() <= 0.0) {
                    while (currentRotation.getDegrees() <= 0.0 && opModeIsActive()) {
                        robot.drive.driveRobotCentric(0.0, 0.0, 2.0);
                        robot.odometry.update();
                        currentRotation = robot.odometry.getPose().getRotation();
                        DebugPartial("correcting rotation: " + currentRotation);

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
