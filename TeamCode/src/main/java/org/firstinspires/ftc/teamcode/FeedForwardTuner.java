package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.teleop.MainPID.horLimitRotations;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.maxSlideSpeed;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.retractFudgeLimitTicks;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.specimenLimitTicks;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.ticksPerRotation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class FeedForwardTuner extends RobotHardware{

    public static double ff = 0.145;

    public static  double ffmax = 0.5;

    public static double ffdifference = ffmax-ff;
    public static boolean enable = true;
    public static double zeroOffset = 1450;
    public static double ticksPerDegreeArmPit = 22.5096444;
    public static double setpoint = 100;

    public static double extensionLimitTicks = -1223;

    public static int ffexpo = 2;


    @Override
    public void loop() {
        super.loop();

        double position = armPitMotor.getCurrentPosition();
        double positionoffset = position + zeroOffset;
        double armangle = positionoffset / ticksPerDegreeArmPit;
        double precentOfExtension = Math.pow(slideMotorRight.getCurrentPosition() / (extensionLimitTicks),ffexpo);

        if (enable) {
            double kg = (ff * Math.cos((Math.toRadians(armangle)))) + (ffdifference * precentOfExtension);
            telemetry.addData("Kg", kg);
            armPitMotor.setPower(kg);
        } else {
            armPitMotor.setPower(0);
        }

        double extensionLimitTicks = (ticksPerRotation * horLimitRotations);

        if (controller1.right_trigger > 0.2) {
            if (!controller1.circle() &&
                    Math.abs( slideMotorRight.getCurrentPosition()) > extensionLimitTicks) {
                slideMotorOut.setPower(0);
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            }
            else if (controller1.triangle() && (
                    Math.abs( slideMotorRight.getCurrentPosition()) > specimenLimitTicks))
            {
                slideMotorOut.setPower(0);
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            } else {
                slideMotorLeft.setPower(controller1.right_trigger * maxSlideSpeed);
                slideMotorRight.setPower(controller1.right_trigger * maxSlideSpeed);
                slideMotorOut.setPower(controller1.right_trigger * maxSlideSpeed);
            }
        } else if (controller1.left_trigger > 0.2) {

            if (controller1.triangle() && (Math.abs( slideMotorRight.getCurrentPosition()) < specimenLimitTicks + retractFudgeLimitTicks))
            {
                slideMotorOut.setPower(0);
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            } else {
                slideMotorLeft.setPower(controller1.left_trigger * -maxSlideSpeed);
                slideMotorRight.setPower(controller1.left_trigger * -maxSlideSpeed);
                slideMotorOut.setPower(controller1.left_trigger * -maxSlideSpeed);
            }
        } else {
            slideMotorOut.setPower(0);
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
        }

        telemetry.addData("arm pitch" , armPitMotor.getCurrentPosition());
        telemetry.addData("position offset", positionoffset);
        telemetry.addData("arm angle", armangle);
        telemetry.addData("extension ticks", slideMotorRight.getCurrentPosition());
    }
}
