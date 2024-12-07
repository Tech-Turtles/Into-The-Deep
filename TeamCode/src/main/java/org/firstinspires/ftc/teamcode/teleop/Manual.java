package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.ARM_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.Constants.ARM_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.Constants.COLOR_SENSOR_STOP_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.RETRACT_FUDGE_LIMIT_TICKS;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_HORIZONTAL_LIMIT_ROTATIONS;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_TICKS_PER_ROTATION;
import static org.firstinspires.ftc.teamcode.Constants.SPECIMEN_LIMIT_TICKS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Manual", group = "B")
@Config
public class Manual extends RobotHardware {
    public static boolean colorSensorStop = true;
    public static double maxIntakeSpeed = 1.0;
    public static double maxSlideSpeed = 1.0;
    public static double maxArmSpeed = 1.0;
    public static double armSetpoint = 0;
    // Max speed in slow mode set to 40%
    private final double slowModeMultiplier = 0.4;
    // Flag to control whether slow mode is on or not
    private boolean slowModeEnabled = false;
    private boolean intakeOn;

    @Override
    public void init() {
        super.init();
        telemetry.addData("Status", "Initialized");
        intakeOn = false;
    }

    @Override
    public void loop() {
        super.loop();

        if (controller1.rightBumper()) {
            if (!colorSensorStop || intakeSensor.getDistance(DistanceUnit.INCH) > COLOR_SENSOR_STOP_DISTANCE) {
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
        } else if (colorSensorStop && intakeSensor.getDistance(DistanceUnit.INCH) < COLOR_SENSOR_STOP_DISTANCE) {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            intakeOn = false;
        }

        if (controller1.right_trigger > 0.2) {
            if (!controller1.circle() &&
                    Math.abs(slideMotorRight.getCurrentPosition()) > getExtensionLimitTicks()) {
                slideMotorOut.setPower(0);
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            } else if (controller1.triangle() && (
                    Math.abs(slideMotorRight.getCurrentPosition()) > getExtensionLimitTicks())) {
                slideMotorOut.setPower(0);
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            } else {
                slideMotorLeft.setPower(controller1.right_trigger * maxSlideSpeed);
                slideMotorRight.setPower(controller1.right_trigger * maxSlideSpeed);
                slideMotorOut.setPower(controller1.right_trigger * maxSlideSpeed);
            }
        } else if (controller1.left_trigger > 0.2) {

            if (controller1.triangle() && (Math.abs(slideMotorRight.getCurrentPosition()) < SPECIMEN_LIMIT_TICKS + RETRACT_FUDGE_LIMIT_TICKS)) {
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

        if (controller1.dpadUp()) {
            if (armPitMotor.getCurrentPosition() > ARM_UPPER_LIMIT) {
                armPitMotor.setPower(0);
            } else
                armPitMotor.setPower(maxArmSpeed);
        } else if (controller1.dpadDown()) {
            if (armPitMotor.getCurrentPosition() < ARM_LOWER_LIMIT) {
                armPitMotor.setPower(0);
            } else
                armPitMotor.setPower(-1.0 * maxArmSpeed);
        } else if (controller1.dpadRight()) {
            if (armPitMotor.getCurrentPosition() < ARM_LOWER_LIMIT) {
                armPitMotor.setPower(0);
            } else
                armPitMotor.setPower(-1.0 * maxArmSpeed / 2.0);
        } else if (controller1.dpadLeft()) {
            if (armPitMotor.getCurrentPosition() > ARM_UPPER_LIMIT) {
                armPitMotor.setPower(0);
            } else
                armPitMotor.setPower(maxArmSpeed / 2.0);
        } else {
            armPitMotor.setPower(0.0);
        }

        // Reset gyro angle if triangle is pressed
        if (controller1.triangleOnce()) {
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

        telemetry.addData("Max Extension Ticks", (SLIDE_TICKS_PER_ROTATION * SLIDE_HORIZONTAL_LIMIT_ROTATIONS));
    }
}