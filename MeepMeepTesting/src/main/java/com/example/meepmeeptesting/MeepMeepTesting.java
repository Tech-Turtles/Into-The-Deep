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
    private static final double robotHalfW = robotH/2.0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.5)
                .setDimensions(robotW, robotH)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        TrajectoryActionBuilder rightStartToSpecimenPlace =
                myBot.getDrive().actionBuilder(new Pose2d(10, (-72 + robotHalfW), Math.toRadians(90)))
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
//                        .splineToConstantHeading(new Vector2d(41, (-26)), Math.toRadians(0))//where to go to scoop sample
//                        .splineToConstantHeading(new Vector2d(45, (-56)), Math.toRadians(-90));

        myBot.runAction(
                new SequentialAction(
                        rightStartToSpecimenPlace.build(),
                        chamberToSpikeMark.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.5f)
                .addEntity(myBot)
                .start();
    }
}