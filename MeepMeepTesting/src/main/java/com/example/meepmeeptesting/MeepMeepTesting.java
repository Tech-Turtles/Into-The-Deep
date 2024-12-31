package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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

    //-------------------------------------------------------------------------
    public static double blue1Angle = 20;
    public static double BLUE_1_X_REL_VALUE = 19;
    public static double BLUE_1_Y_REL_VALUE = -8.5  + 3;
    public static double BLUE_1_DEGREE_REL_VALUE = 0;
    public static double blue1ToHPZoneAngle = -120;
    public static double BLUE_1_X_TO_HPZONE_REL_VALUE = -14;
    public static double BLUE_1_Y_TO_HPZONE_REL_VALUE = -50;
    public static double BLUE_1_DEGREE_TO_HPZONE_REL_VALUE = 140;
    public static double blue2Angle = 30;
    public static double BLUE_2_X_REL_VALUE = 19;
    public static double BLUE_2_Y_REL_VALUE = 19+50;
    public static double BLUE_2_DEGREE_REL_VALUE = -(BLUE_1_DEGREE_TO_HPZONE_REL_VALUE) + 35 -2-2+2-2-2-2;
    public static double BLUE_2_DEGREE_TO_HPZONE_REL_VALUE = -(BLUE_1_DEGREE_TO_HPZONE_REL_VALUE) + 158;
    public static double blue2ToHPZoneAngle = 30;
    public static double blue3Angle = 45;
    public static double BLUE_3_X_REL_VALUE = 19;
    public static double BLUE_3_Y_REL_VALUE = 19+40;
    public static double BLUE_3_DEGREE_REL_VALUE = -(BLUE_2_DEGREE_TO_HPZONE_REL_VALUE) +10-20-5-15+10-3.5+0.5;
    public static double BLUE_3_TOTAL_EXTENSION = -1400;
    public static double BLUE_3_PARTIAL_EXTENSION = 330;
    public static double blue3ToHPZoneAngle = 45;
    public static double BLUE_1_TOTAL_EXTENSION = -670;
    public static double BLUE_1_PARTIAL_EXTENSION = 330;
    public static double BLUE_2_TOTAL_EXTENSION = -900;
    public static double BLUE_2_PARTIAL_EXTENSION = 330;
    public static double blue3Ext = -500;
    public static double SLIDE_CEILING_MAX = 1;
    public static double BLUE_1_SLIDE_CEILING = 0.4;

    //-------------------------------------------------------------------------

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42, 42, Math.toRadians(270), Math.toRadians(270), 11.5)
                .setDimensions(robotW, robotH)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        TrajectoryActionBuilder rightStartToSpecimenPlace =
                myBot.getDrive().actionBuilder(new Pose2d(8.75, (-72 + robotHalfW), Math.toRadians(90)))
                .splineTo(new Vector2d(0 - 5, (-24 - robotHalfW)), Math.toRadians(90));


        TrajectoryActionBuilder chamberToSpikeMark =
                rightStartToSpecimenPlace.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-35))
                        .splineToConstantHeading(new Vector2d(35, -34), Math.toRadians(30)) // intermediate path to not hit the truss
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(35, -22), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(46+4,-18), Math.toRadians(0))
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
                wallIntake.endTrajectory().fresh()
                        .splineToLinearHeading(new Pose2d(59-16, -55+1-4, Math.toRadians(270)), Math.toRadians(-90));

        TrajectoryActionBuilder wallToPlaceSpecimen =
                actualWallIntake.endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(59-16, -50), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0, -40-robotHalfW, Math.toRadians(90.0)), Math.toRadians(180))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0+6, -24-robotHalfW), Math.toRadians(90));

        TrajectoryActionBuilder placeSpecimenToWall =
                wallToPlaceSpecimen.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(0, -30-robotHalfW), Math.toRadians(-90.0))
                        .splineToLinearHeading(new Pose2d(59-16, -55, Math.toRadians(-90.0)), Math.toRadians(0.0));

        TrajectoryActionBuilder wallToActualWall =
                placeSpecimenToWall.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-90.0))
                        .splineToLinearHeading(new Pose2d(59-16, -55+1-4, Math.toRadians(270)), Math.toRadians(-90));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        TrajectoryActionBuilder placeToRotatePoint =
                rightStartToSpecimenPlace.endTrajectory().fresh()
                        .setTangent(Math.toRadians(360))
                        .splineToConstantHeading(new Vector2d(40+5+3.5+0.5, -45), Math.toRadians(0));


        TrajectoryActionBuilder rotatePointBlue1ToHPZone =
                placeToRotatePoint.endTrajectory().fresh()
                        .setTangent(Math.toRadians(0))
                        .turn(Math.toRadians(-BLUE_1_DEGREE_TO_HPZONE_REL_VALUE));

        TrajectoryActionBuilder blue1ToBlue2 =
                rotatePointBlue1ToHPZone.endTrajectory().fresh()
                        .setTangent(Math.toRadians(0))
                        .turn(Math.toRadians(-BLUE_2_DEGREE_REL_VALUE));

        TrajectoryActionBuilder blue2ToHPZone =
                blue1ToBlue2.endTrajectory().fresh()
                        .setTangent(Math.toRadians(0))
                        .turn(Math.toRadians(BLUE_2_DEGREE_REL_VALUE));

        TrajectoryActionBuilder blue2ToBlue3 =
                blue2ToHPZone.endTrajectory().fresh()
                        .setTangent(Math.toRadians(0))
                        .turn(Math.toRadians(BLUE_3_DEGREE_REL_VALUE));

        TrajectoryActionBuilder blue3ToHPZone =
                blue2ToBlue3.endTrajectory().fresh()
                        .setTangent(Math.toRadians(0))
                        .turn(Math.toRadians(-BLUE_3_DEGREE_REL_VALUE));

        TrajectoryActionBuilder blue2ToWallPickUp =
                blue2ToBlue3.endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(40+5+3.5+0.5, -70 + robotHalfW + 5), Math.toRadians(90));

        TrajectoryActionBuilder wallPickUpToSpecPlace1 =
                blue2ToWallPickUp.endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .turn(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(0 -3, (-24 - robotHalfW)), Math.toRadians(160));

        TrajectoryActionBuilder specPlace1ToRotatePoint =
                wallPickUpToSpecPlace1.endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0 -3, (-24 - robotHalfW - 5)), Math.toRadians(90))
                        .turn(Math.toRadians(-180))
                        .setTangent(Math.toRadians(-20))
                        .splineToConstantHeading(new Vector2d(40+5+3.5+0.5, -45), Math.toRadians(0));

        TrajectoryActionBuilder specPlace1RotatePointToWall =
                specPlace1ToRotatePoint.endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(40+5+3.5+0.5, -70 + robotHalfW + 5), Math.toRadians(90));

        TrajectoryActionBuilder wallPickUpToSpecPlace2 =
                specPlace1RotatePointToWall.endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .turn(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(0 +5, (-24 - robotHalfW)), Math.toRadians(160));


//        myBot.runAction(
//                new SequentialAction(
//                        rightStartToSpecimenPlace.build(),
//                        chamberToSpikeMark.build(),
//                        turnAroundAfterPush.build(),
//                        //wallIntake.build(),
//                        actualWallIntake.build(),
//                        wallToPlaceSpecimen.build(),
//                        placeSpecimenToWall.build(),
//                        wallToActualWall.build()
//                       // wallToPlaceSpecimenThird.build()
////                        ,
////                        placeSpecimenToWall.build(),
////                        wallToActualWall.build(),
////                        wallToPlaceSpecimen.build()
//                )

        myBot.runAction(
                new SequentialAction(
                        rightStartToSpecimenPlace.build(),
                        placeToRotatePoint.build(),
                        rotatePointBlue1ToHPZone.build(),
                      blue1ToBlue2.build(),
                        blue2ToHPZone.build(),
                        blue2ToBlue3.build(),
                        //blue3ToHPZone.build()
                        blue2ToWallPickUp.build(),
                        wallPickUpToSpecPlace1.build(),
                        specPlace1ToRotatePoint.build(),
                        specPlace1RotatePointToWall.build(),
                        wallPickUpToSpecPlace2.build()

                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.5f)
                .addEntity(myBot)
                .start();
    }
}