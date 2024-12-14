package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepTesting {

    private static final double robotW = 17.8;
    private static final double robotH = 16;
    private static final double robotHalfW = robotW/2.0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(270), Math.toRadians(270), 11.5)
                .setDimensions(robotW, robotH)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        TrajectoryActionBuilder rightStartToSpecimenPlace =
                myBot.getDrive().actionBuilder(new Pose2d(8.75, (-72 + robotHalfW), Math.toRadians(90)))
                .splineTo(new Vector2d(0, (-24 - robotHalfW)), Math.toRadians(90));


        TrajectoryActionBuilder chamberToSpikeMark =
                rightStartToSpecimenPlace.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-35))
                        .splineToConstantHeading(new Vector2d(35, -34), Math.toRadians(30)) // intermediate path to not hit the truss
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(35, -22), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(46+2, -18), Math.toRadians(0))
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(46+2, -60 + 8), Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(50+3, -14), Math.toRadians(60))
                        .splineToConstantHeading(new Vector2d(59, -20), Math.toRadians(-30))
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(59, -60 + 8), Math.toRadians(-90));

        TrajectoryActionBuilder turnAroundAfterPush =
                chamberToSpikeMark.endTrajectory().fresh()
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(59-16, -55+10), Math.toRadians(180))
                        .turn(Math.toRadians(180));

        TrajectoryActionBuilder wallIntake =
                turnAroundAfterPush.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(59-16, -55+1, Math.toRadians(270)), Math.toRadians(-90));

        TrajectoryActionBuilder actualWallIntake =
                turnAroundAfterPush.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(59-16, -55+1-4, Math.toRadians(270)), Math.toRadians(-90));

        TrajectoryActionBuilder wallToPlaceSpecimen =
                actualWallIntake.endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(59-16, -50), Math.toRadians(90))
                        .turn(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(0, -40-robotHalfW), Math.toRadians(180))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0 + 6, -24-robotHalfW), Math.toRadians(90));



        TrajectoryActionBuilder placeSpecimenToWall =
                wallToPlaceSpecimen.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(0, -30-robotHalfW), Math.toRadians(-90.0))
                        .splineToLinearHeading(new Pose2d(59-16 , -55, Math.toRadians(-90.0)), Math.toRadians(0.0));

        TrajectoryActionBuilder wallToActualWall =
                placeSpecimenToWall.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-90.0))
                        .splineToLinearHeading(new Pose2d(59-16, -55+1-4, Math.toRadians(270)), Math.toRadians(-90));

        myBot.runAction(
                new SequentialAction(
                        rightStartToSpecimenPlace.build(),
                        chamberToSpikeMark.build(),
                        turnAroundAfterPush.build(),
                        //wallIntake.build(),
                        actualWallIntake.build(),
                        wallToPlaceSpecimen.build(),
                        placeSpecimenToWall.build(),
                        wallToActualWall.build(),
                        wallToPlaceSpecimen.build()
//                        ,
//                        placeSpecimenToWall.build(),
//                        wallToActualWall.build(),
//                        wallToPlaceSpecimen.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.5f)
                .addEntity(myBot)
                .start();
    }
}