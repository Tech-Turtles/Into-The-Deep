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
    // Flag to control whether slow mode is on or not
    private boolean slowModeEnabled = false;
    private double maxIntakeSpeed = 1.0;
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

        // Reset gyro angle if triangle is pressed
        if (controller1.triangleOnce()){
            imu.resetYaw();
        }

        // Toggle slow mode on or off if cross is pressed
        if (controller1.crossOnce()) {
            slowModeEnabled = !slowModeEnabled;
        }

        double y = -controller1.left_stick_y;
        double x = controller1.left_stick_x;
        double rx = controller1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Value the motor speeds are multiplied by either 1 or the slow mode percent
        double slowMode = 1.0;
        if (slowModeEnabled)
            slowMode = slowModeMultiplier;

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator * slowMode;
        double rearLeftPower = (rotY - rotX + rx) / denominator * slowMode;
        double frontRightPower = (rotY - rotX - rx) / denominator * slowMode;
        double rearRightPower = (rotY + rotX - rx) / denominator * slowMode;

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