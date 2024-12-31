package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Constants.ARM_D;
import static org.firstinspires.ftc.teamcode.Constants.ARM_I;
import static org.firstinspires.ftc.teamcode.Constants.ARM_P;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_D;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_I;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_P;
import static org.firstinspires.ftc.teamcode.Constants.robotHalfW;
import static org.firstinspires.ftc.teamcode.Constants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.PIDController;

import java.util.function.Supplier;

@Autonomous(group = "A")
public class EXPERIMENTALSpecAuto extends RobotHardware {

    private MecanumDrive drive;
    private Canvas canvas;
    private Action autonomous;
    private double armSetpoint = 0;
    private double slideSetpoint = 0;
    private boolean shouldRun;
    public static double AUTO_LOOP_TIME = 0.042;
    public static boolean SLIDE_PID_ENABLER = true;
    public static double SLIDE_SET_POWER = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////
    public static double BLUE_1_ANGLE = -11.5;
    public static double blue1ToHPZoneAngle = -120;
    public static double blue2Angle = 25;
    public static double blue2ToHPZoneAngle = blue1ToHPZoneAngle + blue2Angle;
    public static double blue3Angle = 45;
    public static double blue3ToHPZoneAngle = 45;
    private double SLIDE_PID_CEILING = 1;
////////////////////////////////////////////////////////////////////////////////////////////////////

    private static double previousTime = 0;

    @Override
    public void init() {
        super.init();

         AUTO_maxWheelVel = 42;
         AUTO_minProfileAccel = -42;
         AUTO_maxProfileAccel = 42;
         AUTO_maxAngVel_Mult = 1.5;
         maxAngAccel_Mult = 1.3;

        armController = new PIDController(ARM_P  +AUTO_ARM_P_FUDGE_FACTOR, ARM_I + AUTO_ARM_I_FUDGE_FACTOR, ARM_D + AUTO_ARM_D_FUDGE_FACTOR, AUTO_LOOP_TIME);
        slideController = new PIDController(SLIDE_P +AUTO_SLIDE_P_FUDGE_FACTOR, SLIDE_I + AUTO_SLIDE_I_FUDGE_FACTOR, SLIDE_D + AUTO_SLIDE_D_FUDGE_FACTOR, AUTO_LOOP_TIME);

        drive = new MecanumDrive(hardwareMap, new Pose2d(8.75, (-72 + robotHalfW), Math.toRadians(90.0)));

        TrajectoryActionBuilder rightStartToSpecimenPlace =
                drive.actionBuilder(new Pose2d(10, (-72 + robotHalfW ), Math.toRadians(90))) // added 6in to line up edge of the robot on the left side of the feild tile perforation
                        .splineTo(new Vector2d(0, (-24 - robotHalfW)), Math.toRadians(90));



        TrajectoryActionBuilder chamberToSpikeMark =
                rightStartToSpecimenPlace.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-35))
                        .splineToConstantHeading(new Vector2d(35, -34), Math.toRadians(30)) // intermediate path to not hit the truss
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(35, -22), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(46+2, -18+8), Math.toRadians(0))
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(46+2, -60 + 8+5 + 5), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(46+2, -20+2+6+5), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(58, -20+2+1+5), Math.toRadians(270))
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(59, -60 + 8 + 5), Math.toRadians(-90));

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
                        .splineToConstantHeading(new Vector2d(59-16, -50-3), Math.toRadians(90))
                        .turn(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(0, -40-robotHalfW), Math.toRadians(180))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0 + 6, -24-robotHalfW), Math.toRadians(90));

        TrajectoryActionBuilder wallToPlaceSpecimenThird =
                actualWallIntake.endTrajectory().fresh()
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(59-16, -50-3), Math.toRadians(90))
                        .turn(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(0, -40-robotHalfW), Math.toRadians(180))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0 + 3, -24-robotHalfW), Math.toRadians(90));



        TrajectoryActionBuilder placeSpecimenToWall =
                wallToPlaceSpecimen.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(0, -30-robotHalfW), Math.toRadians(-90.0))
                        .turn(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(59-16 , -55), Math.toRadians(0.0));

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
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        autonomous = new SequentialAction(
                new InstantAction(() -> //does on start, set arm to spec deposit
                {
                    armSetpoint = Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;
                    slideSetpoint = Constants.HIGH_SPEC_EXT_SLIDE;
                    new SleepAction(0.2);
                }),
                rightStartToSpecimenPlace.build(), //drives to chamber
                new InstantAction(() -> // places spec
                {
                    armSetpoint = Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;
                    slideSetpoint = Constants.SLIDE_SPECIMEN_RETRACT_TICKS; //value inputted, needs to be confirmed
                }),
                new SleepAction(0.25),
                new ParallelAction( // Run the intake & start on the next path
                        new SequentialAction(
                                //outtakes intake to make sure no spec gets stuck in robot
                                new InstantAction(() -> setIntakePower(-1)),
                                new SleepAction(0.3),
                                new InstantAction(() -> setIntakePower(0))
                        ),
                        new SequentialAction( // sets arm horozontal while driving
                                new SleepAction(1.25),
                                new InstantAction(() -> armSetpoint = AUTO_DRIVING_ARM_HOROZONTAL_POSITION)
                        ),
                        placeToRotatePoint.build() //drives to the rotate point
                ),
                new SequentialAction(
                        new InstantAction(() -> setIntakePower( 1.0)),
                        new InstantAction(() -> {
                            slideSetpoint = BLUE_1_TOTAL_EXTENSION + BLUE_1_PARTIAL_EXTENSION;
                            armSetpoint = ARM_HORIZONTAL_POSITION;
                            slideController.setSetpoint(slideSetpoint);
                        }), //extends to the partial point, as to slow extension
                        new ConditionalAction(() -> slideController.atSetpoint()),
                        new InstantAction(()-> {
                            SLIDE_PID_CEILING = BLUE_1_SLIDE_CEILING;
                            slideSetpoint = BLUE_1_TOTAL_EXTENSION;
                            slideController.setSetpoint(slideSetpoint);
                        }),
                        new ConditionalAction(() -> slideController.atSetpoint()),
                        new InstantAction(()->{
                            setIntakePower(0);
                            SLIDE_PID_CEILING = 1;
                            slideSetpoint = BLUE_1_TOTAL_EXTENSION + (BLUE_1_PARTIAL_EXTENSION);
                            armSetpoint = AUTO_DRIVING_ARM_HOROZONTAL_POSITION;
                        })

                ),

                rotatePointBlue1ToHPZone.build(), //turns to deposit sample into human player zone after picking up

                                new SequentialAction(
                                        new InstantAction(() -> setIntakePower(-1.0)),
                                        new SleepAction(0.3)
                                        //new InstantAction(() -> setIntakePower(0))
                                ),
                                blue1ToBlue2.build(),
                                new SequentialAction(
                                        new InstantAction(() -> setIntakePower( 1.0)),
                                        new InstantAction(() -> {
                                            slideSetpoint = BLUE_2_TOTAL_EXTENSION + (BLUE_2_PARTIAL_EXTENSION);
                                            armSetpoint = ARM_HORIZONTAL_POSITION;
                                            slideController.setSetpoint(slideSetpoint);
                                        }), //extends to the partial point, as to slow extension
                                        new ConditionalAction(() -> slideController.atSetpoint()),
                                        new InstantAction(()-> {
                                            SLIDE_PID_CEILING = BLUE_1_SLIDE_CEILING;
                                            slideSetpoint = BLUE_2_TOTAL_EXTENSION;
                                            slideController.setSetpoint(slideSetpoint);
                                        }),
                                        new ConditionalAction(() -> slideController.atSetpoint()),
                                        new InstantAction(()->{
                                            setIntakePower(0);
                                            SLIDE_PID_CEILING = 1;
                                            slideSetpoint = BLUE_1_TOTAL_EXTENSION + (BLUE_1_PARTIAL_EXTENSION);
                                            armSetpoint = AUTO_DRIVING_ARM_HOROZONTAL_POSITION;
                                        }),
                                blue2ToHPZone.build(),
                                        new SequentialAction(
                                                new InstantAction(() -> setIntakePower(-1.0)),
                                                new SleepAction(0.3)
                                                //new InstantAction(() -> setIntakePower(0))
                                        ),
                                        blue2ToBlue3.build(),
                                        new ParallelAction(
                                                new SequentialAction(
                                                new InstantAction(() -> setIntakePower(1.0))
                                                ),
                                                new SequentialAction(
                                                        new InstantAction(() -> {
                                                            armSetpoint = TELE_ARM_WALL_SPEC_INTAKE_ANGLE;
                                                            slideSetpoint = 20;
                                                        })
                                                )
                                        ),
                                blue2ToWallPickUp.build()),
                                new ParallelAction(
                                        new SequentialAction(wallPickUpToSpecPlace1.build()),
                                        new SequentialAction(new InstantAction(() -> setIntakePower(0))),
                                        new SequentialAction(new InstantAction(() -> {
                                                    armSetpoint = ARM_VERTICAL_POSITION;
                                                    slideSetpoint = Constants.HIGH_SPEC_EXT_SLIDE;
                                                })
                                        )
                                ),
                new SequentialAction( // places spec
                        new InstantAction(() -> armSetpoint = Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE),
                        new SleepAction(0.25),
                        new InstantAction(() ->slideSetpoint = Constants.SLIDE_SPECIMEN_RETRACT_TICKS)
                ),
                new SleepAction(0.25),
                new ParallelAction( // Run the intake & start on the next path
                        new SequentialAction(
                                //outtakes intake to make sure no spec gets stuck in robot
                                new InstantAction(() -> setIntakePower(-1)),
                                new SleepAction(0.3),
                                new InstantAction(() -> setIntakePower(0))
                        ),
                        new SequentialAction( // sets arm to wall intake while driving
                                new ParallelAction(
                                        new SequentialAction(new InstantAction(() -> setIntakePower(-1.0))),
                                        specPlace1ToRotatePoint.build(),//drives to the rotate point
                                        new SequentialAction(
                                            new SleepAction(1.5),
                                            new InstantAction(() -> {
                                            setIntakePower(1.0);
                                            armSetpoint = TELE_ARM_WALL_SPEC_INTAKE_ANGLE;
                                             slideSetpoint = 20;})
                                                )
                                )
                        )

                ),

                specPlace1RotatePointToWall.build(),

                wallPickUpToSpecPlace2.build(),

                new SequentialAction(
                        new InstantAction(()-> {
                            armSetpoint = ARM_VERTICAL_POSITION;
                            slideSetpoint = 0;
                        }),
                        new SleepAction(30)
                )


//                                        new SequentialAction(
//                                                new InstantAction(() -> setIntakePower( 1.0)),
//                                                new InstantAction(() -> {
//                                                    slideSetpoint = BLUE_3_TOTAL_EXTENSION + BLUE_3_PARTIAL_EXTENSION;
//                                                    armSetpoint = ARM_HORIZONTAL_POSITION;
//                                                    slideController.setSetpoint(slideSetpoint);
//                                                }), //extends to the partial point, as to slow extension
//                                                new ConditionalAction(() -> slideController.atSetpoint(),2.0),
//                                                new InstantAction(()-> {
//                                                    SLIDE_PID_CEILING = BLUE_1_SLIDE_CEILING;
//                                                    slideSetpoint = BLUE_3_TOTAL_EXTENSION;
//                                                    slideController.setSetpoint(slideSetpoint);
//                                                }),
//                                                new ConditionalAction(() -> slideController.atSetpoint()),
//                                                new InstantAction(()->{
//                                                    setIntakePower(0);
//                                                    SLIDE_PID_CEILING = 1;
//                                                    slideSetpoint = BLUE_1_TOTAL_EXTENSION + (BLUE_1_PARTIAL_EXTENSION );
//                                                    armSetpoint = AUTO_DRIVING_ARM_HOROZONTAL_POSITION;
//                                                }),
//                                                blue3ToHPZone.build(),
//                                                new SequentialAction(
//                                                        new InstantAction(() -> setIntakePower(-1.0)),
//                                                        new SleepAction(0.3),
//                                                        new InstantAction(() -> setIntakePower(0))





//                turnAroundAfterPush.build(), // turning around to intake
//                new InstantAction(() -> //set arm to wall intake position
//                {
//                    armSetpoint = Constants.ARM_WALL_SPEC_INTAKE_ANGLE;
//                    slideSetpoint = Constants.SLIDE_WALL_SPEC_INTAKE_EXT;
//                }),
//                //wallIntake.build(), //drive forward to intake
//                new InstantAction(() -> setIntakePower(1.0)),
//                actualWallIntake.build(),
//                new SleepAction(0.2),
//                new ParallelAction( // Drive to go place the specimen while doing stuff with the arm
//                        wallToPlaceSpecimen.build(), // Start driving to get ready to place
//                        new SequentialAction( // Get the arm ready to place while driving
//                                new InstantAction(() -> {
//                                    armSetpoint = Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;
//                                }),
//                                new SleepAction(0.1),
//                                new InstantAction(() -> setIntakePower(0.0)),
//                                new InstantAction(() -> setIntakePower(-1.0)),
//                                new SleepAction(0.15),
//                                new InstantAction(() -> setIntakePower(0.0)),
//                                new SleepAction(1.5),
//                                new InstantAction(() -> slideSetpoint = Constants.HIGH_SPEC_EXT_SLIDE - 40)
//                        )
//                ),
//                new InstantAction(() -> // places spec
//                {
//                    armSetpoint = Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;
//                    slideSetpoint = Constants.SLIDE_SPECIMEN_RETRACT_TICKS; //value inputted, needs to be confirmed
//                }),
//                new SleepAction(0.2),
//                new ParallelAction(
//                        placeSpecimenToWall.build(),
//                        new SequentialAction(
//                                new SleepAction(1.0),
//                                new InstantAction(() -> //set arm to wall intake position
//                                {
//                                    armSetpoint = Constants.ARM_WALL_SPEC_INTAKE_ANGLE;
//                                    slideSetpoint = Constants.SLIDE_WALL_SPEC_INTAKE_EXT;
//                                })
//                        )
//                ),
//                new InstantAction(() -> setIntakePower(1.0)),
//                wallToActualWall.build(),
//                new ParallelAction( // Drive to go place the specimen while doing stuff with the arm
//                        wallToPlaceSpecimen.build(), // Start driving to get ready to place
//                        new SequentialAction( // Get the arm ready to place while driving
//                                new InstantAction(() -> {
//                                    armSetpoint = Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;
//                                }),
//                                new SleepAction(0.1),
//                                new InstantAction(() -> setIntakePower(0.0)),
//                                new SleepAction(1.5),
//                                new InstantAction(() -> slideSetpoint = Constants.HIGH_SPEC_EXT_SLIDE)
//                        )
//                ),
//                new InstantAction(() -> // places spec
//                {
//                    armSetpoint = Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;
//                    slideSetpoint = Constants.SLIDE_SPECIMEN_RETRACT_TICKS; //value inputted, needs to be confirmed
//                }),
//                new SleepAction(30) // Stay alive
                );
        canvas = new Canvas();
        autonomous.preview(canvas);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        displayData("Arm Encoder Position", armPitMotor.getCurrentPosition());
        displayData("Arm Pit Set Point", (armSetpoint));

        displayData("Slide Position", getSlideEncoderPosition());
        displayData("Slide Extension Setpoint", slideSetpoint);
    }

    @Override
    public void start() {
        super.start();
        shouldRun = true;

    }

    @Override
    public void loop() {
        super.loop();

        displayData("Arm Encoder Position", armPitMotor.getCurrentPosition());
        displayData("Arm Pit Set Point", armSetpoint);

        displayData("Slide Position", getSlideEncoderPosition());
        displayData("Slide Extension Setpoint", slideSetpoint);
        displayData("Slide PID ENABLER", SLIDE_PID_ENABLER);
        displayData("Slide Extension power", slideSetpoint);

        if (shouldRun) {
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            shouldRun = autonomous.run(packet);
            armPitMotor.setPower(calculateArmPower(armSetpoint));
            if (SLIDE_PID_ENABLER){
                double SLIDE_PID = -(slideController.calculate(getSlideEncoderPosition(), slideSetpoint));
                SLIDE_PID = Math.abs(SLIDE_PID) > SLIDE_PID_CEILING ? SLIDE_PID_CEILING * Math.signum(SLIDE_PID) : SLIDE_PID;
                setSlidePower( SLIDE_PID -calculateSlideFeedforward());
            } else {
                setSlidePower(SLIDE_SET_POWER);
            }


        } else {
            armPitMotor.setPower(0.0);
            setSlidePower(0.0);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        }
    }

    public class ConditionalAction implements Action {
        private final Supplier<Boolean> condition;
        private double timeout = -1;
        private double startTime = -1;
        public ConditionalAction(Supplier<Boolean> condition) {
            this.condition = condition;
        }

        public ConditionalAction(Supplier<Boolean> condition, double timeout) {
            this.condition = condition;
            this.timeout = timeout;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean condition = this.condition.get();
            if (condition)
                return false;

            if (timeout > 0) {
                if (startTime < 0) {
                    startTime = now();
                    return true;
                }
                return now() - startTime < timeout;
            }

            return true;
        }
    }

    public double now() {
        return System.nanoTime() * 1e-9;
    }
}
