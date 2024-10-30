package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Manual")
@Config
public class Manual extends RobotHardware {
    public static boolean colorSensorStop = true;
    public static double colorSensorStopDistance = 0.4;
    // Max speed in slow mode set to 40%
    private final double slowModeMultiplier = 0.4;
    public static double maxIntakeSpeed = 1.0;
    public static double maxSlideSpeed = 1.0;
    public static double maxArmSpeed = 1.0;

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

        if (controller1.leftBumper()) {
            if (!colorSensorStop || intakeSensor.getDistance(DistanceUnit.INCH) > colorSensorStopDistance) {
                intakeLeft.setPower(maxIntakeSpeed);
                intakeRight.setPower(maxIntakeSpeed);
                intakeOn = true;
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                intakeOn = false;
            }
        } else if (controller1.rightBumper()) {
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
            slideMotorLeft.setPower(controller1.right_trigger * maxSlideSpeed);
            slideMotorRight.setPower(controller1.right_trigger * maxSlideSpeed);
            slideMotorOut.setPower(controller1.right_trigger * maxSlideSpeed);
        } else if (controller1.left_trigger > 0.2) {
            slideMotorLeft.setPower(controller1.left_trigger * -maxSlideSpeed);
            slideMotorRight.setPower(controller1.left_trigger * -maxSlideSpeed);
            slideMotorOut.setPower(controller1.left_trigger * -maxSlideSpeed);
        } else {
            slideMotorOut.setPower(0);
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
        }

        if (controller1.dpadUp()) {
            armMotorRight.setPower(maxArmSpeed);
        } else if (controller1.dpadDown()) {
            armMotorRight.setPower(-1.0 * maxArmSpeed);
        } else {
            armMotorRight.setPower(0.0);
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

    private void displayTelemetry() {
        telemetry.addData("Front Left Drive Motor Position", frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Drive Motor Position", frontRight.getCurrentPosition());
        telemetry.addData("Rear Left Drive Motor Position", rearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Drive Motor Position", rearRight.getCurrentPosition());

        telemetry.addData("IMU angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addData("Intake Color Sensor Distance (In)", intakeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Intake Color Sensor Colors", "R%.2f, G%.2f, B%.2f, A%.2f",
                intakeSensor.getNormalizedColors().red, intakeSensor.getNormalizedColors().green,
                intakeSensor.getNormalizedColors().blue, intakeSensor.getNormalizedColors().alpha);
    }
}