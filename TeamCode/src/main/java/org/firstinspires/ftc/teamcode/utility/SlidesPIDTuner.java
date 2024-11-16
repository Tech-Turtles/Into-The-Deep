package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.teamcode.FeedForwardTuner.extensionLimitTicks;
import static org.firstinspires.ftc.teamcode.FeedForwardTuner.ffexpo;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.enable;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.ff;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.ffHighAngleCutOff;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.ffLowAngleCutOff;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.ffMax;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.highBucketSampleArmPit;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.highChamberEndArmPit;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.horArmPit;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.maxSlideSpeed;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.retractFudgeLimitTicks;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.specimenLimitTicks;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.ticksPerDegreeArmPit;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.vertArmPit;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.zeroOffset;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp
public class SlidesPIDTuner extends RobotHardware {


    private static double armPitTarget;
    public static boolean slidePowerTester = true;
    public static double slideSetpoint = 0;
    public static double slideP = 0.0;
    public static double slideI = 0.0;
    public static double slideD = 0.0;
    public static double slideffexpo = 0.5;
    public static double slideffBase =0.4;
    public static double slideffMax = 0.58;
    public static double highSpecExtSetPoint = -364;
    public static double highSpecPitSetPoint = 800;
    public static double highSampleExtSetPoint = extensionLimitTicks;
    public static double highSamplePitSetPoint = 650;
    public static double lowSampleExtSetPoint = -500;
    public static double lowSamplePitSetPoint = 500;
    private final PIDController slideController = new PIDController(slideP, slideI, slideD, 0.18);
    private double prevP = slideP, prevI = slideI, prevD = slideD;
    private double slidePIDPower = 0;

    @Override
    public void loop() {
        super.loop();

        if (prevP != slideP || prevI != slideI || prevD != slideD) {
            slideController.setPID(slideP, slideI, slideD);
            prevP = slideP;
            prevI = slideI;
            prevD = slideD;
        }

        double slidePrecentOfExtension = Math.pow(slideMotorRight.getCurrentPosition() / (extensionLimitTicks),slideffexpo);
        double slideffDifference = slideffMax - slideffBase;
        double slideks = (1-slidePrecentOfExtension) * slideffDifference + slideffBase;
        
        if (controller2.dpadLeft()) {
            armPitTarget = highBucketSampleArmPit;
        }

        if (controller2.dpadUp()) {
            armPitTarget = vertArmPit;
        }

        if (controller2.dpadDown()) {
            armPitTarget = horArmPit;
        }

        if (controller2.dpadRight()) {
            armPitTarget = highChamberEndArmPit;
        }

        double position = armPitMotor.getCurrentPosition();
        double positionOffset = position + zeroOffset;
        double armAngle = positionOffset / ticksPerDegreeArmPit;
        double percentOfExtension = Math.pow(slideMotorRight.getCurrentPosition() / (extensionLimitTicks), ffexpo);
        double kg = 0;

        if (enable) {
            double ffDifference = ffMax - ff;

            if (position > ffHighAngleCutOff * ticksPerDegreeArmPit) {
                ffDifference = ffDifference - 0.2;
            }
            kg = (ff * Math.cos((Math.toRadians(armAngle)))) + (ffDifference * percentOfExtension);
            telemetry.addData("Kg", kg);
            telemetry.addData("arm angle", armAngle);

            if (position < ffLowAngleCutOff * ticksPerDegreeArmPit) {
                kg = 0;
            }
        }

        armPitMotor.setPower(armController.calculate(armPitMotor.getCurrentPosition(), armPitTarget) + kg);
        double slidePower = slideks * Math.sin(Math.toRadians(armAngle));

        if (controller2.triangle()){
            slideSetpoint = highSpecExtSetPoint;
            //armPitTarget = highSpecPitSetPoint;
        }

        if (controller2.cross()){
            slideSetpoint = highSampleExtSetPoint;
            //armPitTarget = highSamplePitSetPoint;
        }

        if (controller2.square()){
            slideSetpoint = lowSampleExtSetPoint;
            //armPitTarget = lowSamplePitSetPoint;
        }

        if (controller2.right_trigger > 0.2) {
            if (!controller2.circle() &&
                    Math.abs(slideMotorRight.getCurrentPosition()) > extensionLimitTicks) {
                slideMotorOut.setPower(0);
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            } else {
                slideMotorLeft.setPower(controller2.right_trigger * maxSlideSpeed);
                slideMotorRight.setPower(controller2.right_trigger * maxSlideSpeed);
                slideMotorOut.setPower(controller2.right_trigger * maxSlideSpeed);
            }
        } else if (controller2.left_trigger > 0.2) {
            if (controller2.triangle() && (Math.abs(slideMotorRight.getCurrentPosition()) < specimenLimitTicks + retractFudgeLimitTicks)) {
                slideMotorOut.setPower(0);
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            } else {
                slideMotorLeft.setPower(controller2.left_trigger * -maxSlideSpeed);
                slideMotorRight.setPower(controller2.left_trigger * -maxSlideSpeed);
                slideMotorOut.setPower(controller2.left_trigger * -maxSlideSpeed);
            }
        } else if (controller2.triangle() ){
            slidePIDPower = slideController.calculate(slideMotorRight.getCurrentPosition(), slideSetpoint) + slidePower;
            slideMotorOut.setPower(slidePIDPower);
            slideMotorLeft.setPower(slidePIDPower);
            slideMotorRight.setPower(slidePIDPower);
        } else {
            slideMotorOut.setPower(0);
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
        }

        telemetry.addData("Slide Position", slideMotorRight.getCurrentPosition());
        telemetry.addData("Slide Extension Set point", slideSetpoint);
        telemetry.addData("Slide PID Power", slidePIDPower);
        telemetry.addData("right trigger value", controller2.right_trigger);
        telemetry.addData("left trigger value", controller2.left_trigger);

    }
}
