/*package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.PIDController;

public class PID extends OpMode {
    private PIDController pid = new PIDController(0.0001,0,0);
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        pid.calculate(  );
    }



}*/
package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.FeedForwardTuner.extensionLimitTicks;
import static org.firstinspires.ftc.teamcode.FeedForwardTuner.ffexpo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp(name = "Main PID")
@Config
public class MainPID extends RobotHardware {

    public static double wallSpecimenPit = 200;
    public static boolean colorSensorStop = true;
    public static double colorSensorStopDistance = 0.4;
    // Max speed in slow mode set to 40%
    private final double slowModeMultiplier = 0.4;
    public static double maxIntakeSpeed = 1.0;
    public static double maxSlideSpeed = 1.0;
    public static double maxArmSpeed = 1.0;
    public static double ticksPerRotation = 537.7 ;
    public static double horLimitRotations = 1.55;
    public static double specimenLimitTicks = 364;
    public static double retractFudgeLimitTicks = 20;
    public static double armLowerLimit = -1667;
    public static double armUpperLimit = 950 ;
    private static double armPitRiseTime = 1; //seconds, not firsts
    private static double loopRate =  100; //Hz
    public static double armPitTotalEncoderTicks = Math.abs(armLowerLimit) + armUpperLimit;
    private static double armPitPidStep = (armPitTotalEncoderTicks / (armPitRiseTime * loopRate));
    public static double highBucketSampleArmPit = -500;
    public static double vertArmPit = 700;
    public static double horArmPit = -1500;
    public static double highChamberEndArmPit = -50;
    public static boolean enable = true;
    public static double ff = 0.145;
    public static  double ffMax = 0.3;
    public static double zeroOffset = 1450;
    public static double ticksPerDegreeArmPit = 22.5096444;
    public static double ffHighAngleCutOff = 90 ;
    public static double ffLowAngleCutOff = -10 ;
    public static double ffCuttoffCoEff= 0.4;
    private double armPitTarget = 0;
    // Flag to control whether slow mode is on or not
    private boolean slowModeEnabled = false;
    private boolean intakeOn;
    private double prevP = pCoeffPID, prevI = iCoeffPID, prevD = dCoeffPID;
    public static boolean slidePowerTester = true;
    public static double slideSetpoint = 0;
    public static double slideP = 0.028;
    public static double slideI = 0.028;
    public static double slideD = 0.00004;
    public static double slideffexpo = 0.5;
    public static double slideffBase =0.4;
    public static double slideffMax = 0.58;
    public static double highSpecExtSetPoint = -364;
    public static double highSpecPitSetPoint = 800;
    public static double highSampleExtSetPoint = extensionLimitTicks;
    public static double highSamplePitSetPoint = 650;
    public static double lowSampleExtSetPoint = -500;
    public static double lowSamplePitSetPoint = 500;
    private final PIDController slideController = new PIDController(slideP, slideI, slideD, 0.018);
    private double prevSlideP = slideP, prevSlideI = slideI, prevSlideD = slideD;
    private double slidePIDPower = 0;

    @Override
    public void init() {
        super.init();
        telemetry.addData("Status", "Initialized");
        intakeOn = false;
    }

    @Override
    public void loop() {
        super.loop();

        if (prevP != pCoeffPID || prevI != iCoeffPID || prevD != dCoeffPID) {
            armController.setPID(pCoeffPID, iCoeffPID, dCoeffPID);
            armController.reset();
            prevP = pCoeffPID;
            prevI = iCoeffPID;
            prevD = dCoeffPID;
        }

        double extensionLimitTicks = (ticksPerRotation * horLimitRotations);

        if (controller2.rightBumper()){
            if (!colorSensorStop || intakeSensor.getDistance(DistanceUnit.INCH) > colorSensorStopDistance) {
                intakeLeft.setPower(maxIntakeSpeed);
                intakeRight.setPower(maxIntakeSpeed);
                intakeOn = true;
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                intakeOn = false;
            }
        } else if (controller2.leftBumper()) {
            intakeLeft.setPower(-maxIntakeSpeed);
            intakeRight.setPower(-maxIntakeSpeed);
            intakeOn = false;
        } else if (!intakeOn) {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        } else if (colorSensorStop && intakeSensor.getDistance(DistanceUnit.INCH) < colorSensorStopDistance) {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            intakeOn = false;
        }

        if (prevSlideP != slideP || prevSlideI != slideI || prevSlideD != slideD) {
            slideController.setPID(slideP, slideI, slideD);
            slideController.reset();
            prevSlideP = slideP;
            prevSlideI = slideI;
            prevSlideD = slideD;
        }

        double slidePrecentOfExtension = Math.pow(slideMotorRight.getCurrentPosition() / (extensionLimitTicks),slideffexpo);
        if (Double.isNaN(slidePrecentOfExtension))
        {
            slidePrecentOfExtension = 0.0;
        }
        double slideffDifference = slideffMax - slideffBase;
        double slideks = slidePrecentOfExtension * slideffDifference + slideffBase;


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
        double kg = 0;
        double percentOfExtension = Math.pow(slideMotorRight.getCurrentPosition() / (extensionLimitTicks), ffexpo);

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

        if (Double.isNaN(slidePower)) {
            slideMotorOut.setPower(0);
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
        }

        if (controller2.triangle()){
            slideSetpoint = highSpecExtSetPoint;
            armPitTarget = highSpecPitSetPoint;
        }

        if (controller2.cross()){
            slideSetpoint = highSampleExtSetPoint;
            armPitTarget = highSamplePitSetPoint;
        }

        if (controller2.square()){
            slideSetpoint = lowSampleExtSetPoint;
            armPitTarget = lowSamplePitSetPoint;
        }


        if (controller2.right_trigger > 0.2) {
          /*
            if (!controller2.circle() &&
                    Math.abs(slideMotorRight.getCurrentPosition()) > extensionLimitTicks) {
                slideMotorOut.setPower(0);
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            } else {
            */
            slideMotorLeft.setPower(controller2.right_trigger * maxSlideSpeed);
            slideMotorRight.setPower(controller2.right_trigger * maxSlideSpeed);
            slideMotorOut.setPower(controller2.right_trigger * maxSlideSpeed);
            //}
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
        } else if (controller2.triangle() || controller2.square() || controller2.cross() ){
            double pid = slideController.calculate(slideMotorRight.getCurrentPosition(), slideSetpoint);
            slidePIDPower = -(pid + slidePower);
            slideMotorOut.setPower(slidePIDPower);
            slideMotorLeft.setPower(slidePIDPower);
            slideMotorRight.setPower(slidePIDPower);
            dashboardTelemetry.addData("pid", pid);
        } else {
            slideMotorOut.setPower(0);
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
        }

        // Reset gyro angle if triangle is pressed
        if (controller1.triangleOnce()){
            imu.resetYaw();
        }

        // Toggle slow mode on or off if cross is pressed
        if (controller1.crossOnce()) {
            slowModeEnabled = !slowModeEnabled;
        }

        double y = Math.pow(-controller1.left_stick_y, 3);
        double x = Math.pow(controller1.left_stick_x * 1.1, 3);
        double rx = Math.pow(controller1.right_stick_x, 3);

        // Value the motor speeds are multiplied by either 1 or the slow mode percent
        double slowMode = 1.0;
        if (slowModeEnabled)
            slowMode = slowModeMultiplier;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator * slowMode;
        double rearLeftPower = (y - x + rx) / denominator * slowMode;
        double frontRightPower = (y - x - rx) / denominator * slowMode;
        double rearRightPower = (y + x - rx) / denominator * slowMode;

        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

        displayTelemetry();
    }

    private void displayTelemetry()

    {
        telemetry.addData("Front Left Drive Motor Position", frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Drive Motor Position", frontRight.getCurrentPosition());
        telemetry.addData("Rear Left Drive Motor Position", rearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Drive Motor Position", rearRight.getCurrentPosition());

        telemetry.addData("Extension Left Encoder Position", slideMotorLeft.getCurrentPosition());
        telemetry.addData("Extension Right Encoder Position", slideMotorRight.getCurrentPosition());

        telemetry.addData("Arm Encoder Position", armPitMotor.getCurrentPosition());

        telemetry.addData("IMU angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addData("Intake Color Sensor Distance (In)", intakeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Intake Color Sensor Colors", "R%.2f, G%.2f, B%.2f, A%.2f",
                intakeSensor.getNormalizedColors().red, intakeSensor.getNormalizedColors().green,
                intakeSensor.getNormalizedColors().blue, intakeSensor.getNormalizedColors().alpha);


        telemetry.addData("Max Extension Ticks", (ticksPerRotation * horLimitRotations));
        telemetry.addData("Arm Pit Set Point", (armPitTarget));
        telemetry.addData("Arm Pit motor power", (armController.calculate(armPitMotor.getCurrentPosition(), armPitTarget)));
        telemetry.addData("Arm Pit PID Steps", (armPitPidStep));
        telemetry.addData("extension power", ((controller2.right_trigger * maxSlideSpeed)));

        dashboardTelemetry.addData("Slide PID Power", slidePIDPower);
        dashboardTelemetry.addData("Slide Position", slideMotorRight.getCurrentPosition());
        dashboardTelemetry.addData("Slide Extension Set point", slideSetpoint);
        dashboardTelemetry.addData("right trigger value", controller2.right_trigger);
        dashboardTelemetry.addData("left trigger value", controller2.left_trigger);
        dashboardTelemetry.addData("slideP",slideP);
        dashboardTelemetry.addData("slideI", slideI);
        dashboardTelemetry.addData("slideD", slideD);

    }
}
