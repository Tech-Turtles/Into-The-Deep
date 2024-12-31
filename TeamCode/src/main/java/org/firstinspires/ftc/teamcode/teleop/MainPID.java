package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.ARM_BUCKET_SAMPLE_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_CHAMBER_END_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_SAMPLE_PIVOT_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_SPEC_PIVOT_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HORIZONTAL_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_LOW_SAMPLE_PIVOT_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_VERTICAL_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.COLOR_SENSOR_STOP_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_SLOW_MODE_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.Constants.HIGH_SAMPLE_EXT_SLIDE;
import static org.firstinspires.ftc.teamcode.Constants.HIGH_SPEC_EXT_SLIDE;
import static org.firstinspires.ftc.teamcode.Constants.HIGH_SPEC_WALL_EXT_SLIDE;
import static org.firstinspires.ftc.teamcode.Constants.LOW_SAMPLE_EXT_SLIDE;
import static org.firstinspires.ftc.teamcode.Constants.TELE_ARM_WALL_SPEC_INTAKE_ANGLE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Main PID", group = "A")
@Config
public class MainPID extends RobotHardware {

    public static boolean colorSensorStop = true;
    public static double maxIntakeSpeed = 1.0;
    public static double maxSlideSpeed = 1.0;
    public static double slideSetpoint = 0;
    // Flag to control whether slow mode is on or not
    private boolean slowModeEnabled = false;
    private boolean intakeOn;
    private double armPitTarget = 0;

    @Override
    public void init() {
        super.init();
        telemetry.addData("Status", "Initialized");
        intakeOn = false;
    }

    @Override
    public void loop() {
        super.loop();
//        if (controller1.leftBumper()) {
//            if (!colorSensorStop || intakeSensor.getDistance(DistanceUnit.INCH) > COLOR_SENSOR_STOP_DISTANCE) {
//                setIntakePower(maxIntakeSpeed);
//                intakeOn = true;
//            } else {
//                setIntakePower(0);
//                intakeOn = false;
//            }
//        } else if (controller1.rightBumper()) {
//            setIntakePower(-maxIntakeSpeed);
//            intakeOn = false;
//        } else if (!intakeOn) {
//            setIntakePower(0);
//        } else if (colorSensorStop && intakeSensor.getDistance(DistanceUnit.INCH) < COLOR_SENSOR_STOP_DISTANCE) {
//            setIntakePower(0);
//            intakeOn = false;
//        }
        if (controller2.rightBumper()) {
            intakeOn = !shouldStopIntake();
            setIntakePower(intakeOn ? maxIntakeSpeed : 0);
        } else if (controller2.leftBumper()) {
            intakeOn = false;
            setIntakePower(-maxIntakeSpeed);
        } else if (!intakeOn || shouldStopIntake()) {
            setIntakePower(0);
        }

        if (controller2.dpadLeft()) {
            armPitTarget = ARM_BUCKET_SAMPLE_ANGLE;
        }

        if (controller2.dpadUp()) {
            armPitTarget = ARM_VERTICAL_POSITION;

        }

        if (controller2.dpadDown()) {
            armPitTarget = ARM_HORIZONTAL_POSITION;
        }

        if (controller2.dpadRight()) {
            armPitTarget = ARM_HIGH_CHAMBER_END_POSITION;
        }

        if (controller2.rightStickButton()){
            setIntakePower(0.0);
            armPitTarget = ARM_HIGH_SPEC_PIVOT_ANGLE;
            slideSetpoint = HIGH_SPEC_WALL_EXT_SLIDE + 36;
        }

        if (controller2.leftStickButton()){
            intakeOn = true;
            setIntakePower(1.0);
            armPitTarget = TELE_ARM_WALL_SPEC_INTAKE_ANGLE;
            slideSetpoint = 20;
        }

        if (controller2.triangle()) {
            slideSetpoint = HIGH_SPEC_EXT_SLIDE + 36;
            armPitTarget = ARM_HIGH_SPEC_PIVOT_ANGLE;
        }

        if (controller2.cross()) {
            slideSetpoint = HIGH_SAMPLE_EXT_SLIDE;
            armPitTarget = ARM_HIGH_SAMPLE_PIVOT_ANGLE;
        }

        if (controller2.square()) {
            slideSetpoint = LOW_SAMPLE_EXT_SLIDE;
            armPitTarget = ARM_LOW_SAMPLE_PIVOT_ANGLE;
        }


        if (controller2.circle()){
            armPitTarget = ARM_HIGH_SPEC_PLACE_PIVOT_ANGLE;

        }

        double armPower = calculateArmPower(armPitTarget);
        displayData("Arm Power", armPower);
        armPitMotor.setPower(armPower);

        // PID + FF
        double slidePower = -(slideController.calculate(getSlideEncoderPosition(), slideSetpoint) + calculateSlideFeedforward());
        displayData("Slide Power", slidePower);

        if (controller2.right_trigger > 0.2) {
            if (!controller2.circle()
                    && Math.abs(slideMotorRight.getCurrentPosition()) > getExtensionLimitTicks()) {
                setSlidePower(0);
            } else {
                setSlidePower(controller2.right_trigger * maxSlideSpeed);
            }
        } else if (controller2.left_trigger > 0.2) {
            setSlidePower(controller2.left_trigger * -maxSlideSpeed);
        } else if (controller2.triangle()
                || controller2.square()
                || controller2.cross()
                || controller2.rightStickButton()
                || controller2.leftStickButton()
        ){
            setSlidePower(slidePower);
        } else {
            setSlidePower(0);
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
            slowMode = DRIVE_SLOW_MODE_MULTIPLIER;

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

    private boolean shouldStopIntake() {
        return colorSensorStop && intakeSensor.getDistance(DistanceUnit.INCH) < COLOR_SENSOR_STOP_DISTANCE;
    }

    private void displayTelemetry() {
        displayData("Front Left Drive Motor Position", frontLeft.getCurrentPosition());
        displayData("Front Right Drive Motor Position", frontRight.getCurrentPosition());
        displayData("Rear Left Drive Motor Position", rearLeft.getCurrentPosition());
        displayData("Rear Right Drive Motor Position", rearRight.getCurrentPosition());

        displayData("Extension Right Encoder Position", slideMotorRight.getCurrentPosition());

        displayData("Arm Encoder Position", armPitMotor.getCurrentPosition());

        displayData("IMU angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        displayData("Intake Color Sensor Distance (In)", intakeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Intake Color Sensor Colors", "R%.2f, G%.2f, B%.2f, A%.2f",
                intakeSensor.getNormalizedColors().red, intakeSensor.getNormalizedColors().green,
                intakeSensor.getNormalizedColors().blue, intakeSensor.getNormalizedColors().alpha);


        displayData("Max Extension Ticks", getExtensionLimitTicks());
        displayData("Arm Pit Set Point", (armPitTarget));

        displayData("Slide Position", getSlideEncoderPosition());
        displayData("Slide Extension Setpoint", slideSetpoint);

    }
}