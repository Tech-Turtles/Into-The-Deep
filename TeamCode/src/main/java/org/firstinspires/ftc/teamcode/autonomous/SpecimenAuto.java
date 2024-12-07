package org.firstinspires.ftc.teamcode.autonomous;

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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(group = "A")
public class SpecimenAuto extends RobotHardware {

    private MecanumDrive drive;
    private Canvas canvas;
    private Action autonomous;
    private double armSetpoint = 0;
    private double slideSetpoint = 0;
    private boolean finished;

    @Override
    public void init() {
        super.init();

        drive = new MecanumDrive(hardwareMap, new Pose2d(10, (-72 + robotHalfW), Math.toRadians(90.0)));

        TrajectoryActionBuilder rightStartToSpecimenPlace =
                drive.actionBuilder(new Pose2d(10, (-72 + robotHalfW), Math.toRadians(90)))
                        .splineTo(new Vector2d(0, (-24 - robotHalfW)), Math.toRadians(90));


        TrajectoryActionBuilder chamberToSpikeMark =
                rightStartToSpecimenPlace.endTrajectory().fresh()
                        .setTangent(Math.toRadians(-35))
                        .splineToConstantHeading(new Vector2d(35, -34), Math.toRadians(30))
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

        autonomous = new SequentialAction(
                new InstantAction(() ->
                {
                    armSetpoint = Constants.ARM_HIGH_CHAMBER_END_POSITION;
                    slideSetpoint = Constants.HIGH_SPEC_EXT_SLIDE;
                }),
                new SleepAction(1.0),
                rightStartToSpecimenPlace.build(),
                new InstantAction(() ->
                {
                    armSetpoint = Constants.ARM_HIGH_CHAMBER_END_POSITION;
                    slideSetpoint = Constants.SLIDE_SPECIMEN_RETRACT_TICKS; //ToDo find value
                }),
                new SleepAction(1.0),
                chamberToSpikeMark.build()
        );

        canvas = new Canvas();
        autonomous.preview(canvas);
    }

    @Override
    public void start() {
        super.start();
        finished = false;
    }

    @Override
    public void loop() {
        super.loop();

        if (!finished) {
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            finished = autonomous.run(packet);
            dashboard.sendTelemetryPacket(packet);

            armPitMotor.setPower(calculateArmPower(armSetpoint));
            setSlidePower(-(slideController.calculate(getSlideEncoderPosition(), slideSetpoint) + calculateSlideFeedforward()));
        } else {
            armPitMotor.setPower(0.0);
            setSlidePower(0.0);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        }
    }
}
