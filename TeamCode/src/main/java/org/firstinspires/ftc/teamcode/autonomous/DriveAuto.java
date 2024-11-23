package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Drive Autonomous")
public class DriveAuto extends LinearOpMode {
    private static final double robotW = 17.8;
    private static final double robotH = 16;
    private static final double robotHalfW = robotW/2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(10, (-72 + robotHalfW), Math.toRadians(90.0)));

        TrajectoryActionBuilder rightStartToSpecimenPlace =
                drive.actionBuilder(new Pose2d(10, (-72 + robotHalfW), Math.toRadians(90)))
                        .splineTo(new Vector2d(0, (-24 - robotHalfW)), Math.toRadians(90));


        TrajectoryActionBuilder chamberToSpikeMark =
                rightStartToSpecimenPlace.endTrajectory().fresh() //specimen place
                        .setTangent(Math.toRadians(-35))
                        .splineToConstantHeading(new Vector2d(35, -34), Math.toRadians(30)) // intermediate path to not hit the truss
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(35, -22), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(46, -18), Math.toRadians(0))
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(46, -60), Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(50, -14), Math.toRadians(60))
                        .splineToConstantHeading(new Vector2d(56, -20), Math.toRadians(-30))
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(56, -60), Math.toRadians(-90));

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(rightStartToSpecimenPlace.build(), chamberToSpikeMark.build()));
    }
}
