package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Constants.ARM_D;
import static org.firstinspires.ftc.teamcode.Constants.ARM_I;
import static org.firstinspires.ftc.teamcode.Constants.ARM_P;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_D;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_I;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_P;
import static org.firstinspires.ftc.teamcode.Constants.robotHalfW;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@Autonomous(group = "A")
public class SpecimenAuto extends RobotHardware {

    private MecanumDrive drive;
    private Canvas canvas;
    private Action autonomous;
    private double armSetpoint = 0;
    private double slideSetpoint = 0;
    private boolean shouldRun;
    public static double autoLoopTime = 0.035;

    private static double previousTime = 0;

    @Override
    public void init() {
        super.init();

        armController = new PIDController(ARM_P, ARM_I, ARM_D, autoLoopTime);
        slideController = new PIDController(SLIDE_P, SLIDE_I, SLIDE_D, autoLoopTime);

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
                        .splineToConstantHeading(new Vector2d(59-6, -55+10), Math.toRadians(-90))
                        .turn(Math.toRadians(180))
                        .setTangent(Math.toRadians(90));

        TrajectoryActionBuilder wallIntake =
                turnAroundAfterPush.endTrajectory().fresh()
                        .splineToLinearHeading(new Pose2d(59-6, -55+1, Math.toRadians(270)), Math.toRadians(90))
                        .setTangent(Math.toRadians(90));

        TrajectoryActionBuilder actualWallIntake =
                wallIntake.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(59-6, -55+1-4, Math.toRadians(270)), Math.toRadians(90))
                .setTangent(Math.toRadians(90));


        autonomous = new SequentialAction(
                new InstantAction(() -> //does on start, set arm to spec deposit
                {
                    armSetpoint = Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;
                    slideSetpoint = Constants.HIGH_SPEC_EXT_SLIDE;
                }),
                new SleepAction(1.0),
                rightStartToSpecimenPlace.build(), //drives to chamber
                new InstantAction(() -> // places spec
                {
                    armSetpoint = Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;
                    slideSetpoint = Constants.SLIDE_SPECIMEN_RETRACT_TICKS; //value inputed, needs to be confirmed
                }),
                new SleepAction(0.5),//outtakes intake to make sure no spec gets stuck in robot
                new InstantAction(() ->{
                    setIntakePower(-1);
                }),
                new SleepAction(0.3),
                new InstantAction(() ->{
                    setIntakePower(0);
                }),
                chamberToSpikeMark.build(), //pushing spline
                turnAroundAfterPush.build(), // turning aroounf to intake
        new InstantAction(() ->{ //set arm to wall intake position
            armSetpoint = Constants.ARM_WALL_SPEC_INTAKE_ANGLE;
            slideSetpoint = Constants.SLIDE_WALL_SPEC_INTAKE_EXT;
        }),
                wallIntake.build(), //drive forward to intake
        new SleepAction(2.0),
                actualWallIntake.build()

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

        if (shouldRun) {
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            shouldRun = autonomous.run(packet);
            armPitMotor.setPower(calculateArmPower(armSetpoint));
            setSlidePower(-(slideController.calculate(getSlideEncoderPosition(), slideSetpoint) + calculateSlideFeedforward()));
            displayData("Arm Encoder Position", armPitMotor.getCurrentPosition());
            displayData("Arm Pit Set Point", (armSetpoint));

            displayData("Slide Position", getSlideEncoderPosition());
            displayData("Slide Extension Setpoint", slideSetpoint);


        } else {
            armPitMotor.setPower(0.0);
            setSlidePower(0.0);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
            displayData("finished", true);
        }
    }
}
