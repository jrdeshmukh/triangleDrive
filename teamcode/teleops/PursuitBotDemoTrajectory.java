package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;

import java.util.ArrayList;

// pure pursuit algorithm demo
@TeleOp(name="PursuitBotDemoTrajectory", group="PursuitBot")
public class PursuitBotDemoTrajectory extends LinearOpMode {

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
    public double maxVelocity = 0.5;
    public double maxAcceleration = 0.5;

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
                        getInputX(), getInputY(), getInputRot());

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

            TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    robot.odometry.getPose(), new ArrayList<>(), new Pose2d(), config);

            // create start and end waypoints from current pose to origin pose
            double totalTime = trajectory.getTotalTimeSeconds();
            Waypoint[] point = new Waypoint[100];
            for (int i = 1; i < point.length - 1; i++) {
                Pose2d pose = trajectory.sample((double)i / point.length * totalTime).poseMeters;
                point[i] = new GeneralWaypoint(pose, movementSpeed, turnSpeed, followRadius);
            }

            point[0] = new StartWaypoint(trajectory.sample(0).poseMeters);
            point[point.length - 1] = new EndWaypoint(trajectory.sample(totalTime).poseMeters,
                    movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

            // follow path formed by waypoints
            PurePursuitCommand command = new PurePursuitCommand(
                    robot.drive, robot.odometry, point);
            RunCommand(command, "return home");
        }
    }

    // run command linearly
    public void RunCommand(PurePursuitCommand command, String state) {

        // follow path
        command.schedule();

        // loop while following
        while (opModeIsActive() && !command.isFinished()) {

            robot.odometry.update();
            command.execute();
            DebugFull(state);
        }

        // end robot movement
        command.end(true);
        robot.drive.stop();

        // wait a second
        if (opModeIsActive()) sleep(1000);
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
        telemetry.addData("input horizontal", getInputX());
        telemetry.addData("input vertical", getInputY());
        telemetry.addData("input rotational", getInputRot());
        telemetry.update();
    }

    public float getInputX() {

        return -gamepad1.left_stick_y;
    }

    public float getInputY() {

        return gamepad1.left_stick_x;
    }

    public float getInputRot() {

        return gamepad1.right_stick_x;
    }
}
