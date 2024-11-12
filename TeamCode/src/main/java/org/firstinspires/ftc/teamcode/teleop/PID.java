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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp(name = "PID")
@Config
public class PID extends RobotHardware {
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

    public static double armPitRiseTime = 1; //seconds, not firsts

    public static double loopRate =  100; //Hz

    public static double armPitTotalEncoderTicks = Math.abs(armLowerLimit) + armUpperLimit;

    public static double armPitPidStep = (armPitTotalEncoderTicks / (armPitRiseTime * loopRate));
    /*
    time to get all the way (in sec)
    rate that the loop executs at (200 updates /sec? )
    endoder tics for all the way
    amount to add on

     */

    public static double pCoeffPID = 0.0015;

    public static double iCoeffPID = 0.00;

    public static double dCoeffPID = 0.00006;

    public static double highBucketSampleArmPit = 650;

    public static double vertArmPit = 800;

    public static double horArmPit = -1500;

    public static double highChamberEndArmPit = -50;
    private PIDController armPitPID = new PIDController(pCoeffPID,iCoeffPID,dCoeffPID);


    private double armPitTarget = 0;




    // Flag to control whether slow mode is on or not
    private boolean slowModeEnabled = false;
    private boolean intakeOn;

    private double prevP = pCoeffPID, prevI = iCoeffPID, prevD = dCoeffPID;

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
            armPitPID.setPID(pCoeffPID, iCoeffPID, dCoeffPID);
            armPitPID.reset();
            prevP = pCoeffPID;
            prevI = iCoeffPID;
            prevD = dCoeffPID;
        }

        double extensionLimitTicks = (ticksPerRotation * horLimitRotations);

        if (controller1.rightBumper()){
            if (!colorSensorStop || intakeSensor.getDistance(DistanceUnit.INCH) > colorSensorStopDistance) {
                intakeLeft.setPower(maxIntakeSpeed);
                intakeRight.setPower(maxIntakeSpeed);
                intakeOn = true;
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                intakeOn = false;
            }
        } else if (controller1.leftBumper()) {
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
        /*
         declare arm target V
         increase arm target when dpad up,
         decrease arm target when dpad down

         make sure taget dosent go ourside of motor upplimit or motorlowerlimitV


         pid calculate using motor target and the encoder
         dpadup (set speed motorspeed = pid calculate)

        */

 /*
        if (controller1.dpadUp()) {
            if (armPitMotor.getCurrentPosition() > armUpperLimit) {
                armPitMotor.setPower(0);
            } else {
                armPitMotor.setPower(maxArmSpeed);
                armPitTarget = armPitMotor.getCurrentPosition();
            }
        } else {
            armPitMotor.setPower(armPitPID.calculate(armPitMotor.getCurrentPosition(), armPitTarget));
        }
*/

        if (controller1.dpadLeft()) {
            armPitTarget = highBucketSampleArmPit;
        }

        if (controller1.dpadUp()){
            armPitTarget = vertArmPit;
        }

        if (controller1.dpadDown()){
            armPitTarget = horArmPit;
        }

       if (controller1.dpadRight()){
           armPitTarget = highChamberEndArmPit;


        }

       /*
        if (controller1.dpadUp()) {
            armPitTarget = armPitTarget + armPitPidStep;
            if (armPitTarget > armUpperLimit) {
                armPitTarget = armUpperLimit;
            }
        }

        if (controller1.dpadLeft()) {
            armPitTarget = armPitTarget + (armPitPidStep / 2);
            if (armPitTarget > armUpperLimit) {
                armPitTarget = armUpperLimit;
            }
        }

        if (controller1.dpadDown()) {
            armPitTarget = armPitTarget - armPitPidStep;
            if (armPitTarget < armLowerLimit) {
                armPitTarget = armLowerLimit;
            }
        }

        if (controller1.dpadRight()) {
            armPitTarget = armPitTarget - (armPitPidStep / 2);
            if (armPitTarget < armLowerLimit) {
                armPitTarget = armLowerLimit;
            }
        }
 */
        armPitMotor.setPower(armPitPID.calculate(armPitMotor.getCurrentPosition(), armPitTarget));


        /*
        if (controller1.dpadUp()) {
            if  (armPitMotor.getCurrentPosition() > armUpperLimit)
            {
                armPitMotor.setPower(0);
            }
            else
                armPitMotor.setPower(maxArmSpeed);
        } else if (controller1.dpadDown()) {
            if  (armPitMotor.getCurrentPosition() < armLowerLimit)
            {
                armPitMotor.setPower(0);
            }
            else
                armPitMotor.setPower(-1.0 * maxArmSpeed);
        } else if (controller1.dpadRight()) {
            if  (armPitMotor.getCurrentPosition() < armLowerLimit)
            {
                armPitMotor.setPower(0);
            }
            else
                armPitMotor.setPower(-1.0 * maxArmSpeed / 2.0);
        } else if (controller1.dpadLeft()) {
            if  (armPitMotor.getCurrentPosition() > armUpperLimit)
            {
                armPitMotor.setPower(0);
            }
            else
                armPitMotor.setPower( maxArmSpeed / 2.0);
        } else {
            armPitMotor.setPower(0.0);
        } */

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

    private void displayTelemetry() {
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
        telemetry.addData("Arm Pit motor power", (armPitPID.calculate(armPitMotor.getCurrentPosition(), armPitTarget)));
        telemetry.addData("Arm Pit PID Steps", (armPitPidStep));
    }
}
