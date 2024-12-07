package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ARM_D;
import static org.firstinspires.ftc.teamcode.Constants.ARM_FEEDFORWARD;
import static org.firstinspires.ftc.teamcode.Constants.ARM_FEEDFORWARD_EXPONENT;
import static org.firstinspires.ftc.teamcode.Constants.ARM_FEEDFORWARD_MAX;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_ANGLE_FEEDFORWARD_CUTOFF;
import static org.firstinspires.ftc.teamcode.Constants.ARM_I;
import static org.firstinspires.ftc.teamcode.Constants.ARM_LOW_ANGLE_FEEDFORWARD_CUTOFF;
import static org.firstinspires.ftc.teamcode.Constants.ARM_P;
import static org.firstinspires.ftc.teamcode.Constants.ARM_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_ZERO_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_D;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_FEEDFORWARD_BASE;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_FEEDFORWARD_EXPONENT;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_FEEDFORWARD_MAX;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_HORIZONTAL_LIMIT_ROTATIONS;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_I;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_P;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_TICKS_PER_ROTATION;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.Controller;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.PIDController;

/**
 * RobotHardware manages all hardware components of the robot, such as motors, servos, sensors,
 * and PID controllers. It provides methods to initialize, control, and monitor the hardware.
 * </br>
 * This class also interacts with the FTC Dashboard for debugging and telemetry data.
 * </br>
 * Major Components:
 * - Drive motors (front/rear, left/right)
 * - Arm mechanism (arm motor and PID control)
 * - Slide mechanism (multiple slide motors and PID control)
 * - Intake system (servos and color sensor)
 * - IMU for orientation
 */

@Config
public class RobotHardware extends OpMode {

    // ---------------------------------------------------
    // Dashboard variables
    public static boolean useFeedforwardCutoff = false;
    protected  PIDController armController = new PIDController(ARM_P, ARM_I, ARM_D, 0.018);
    protected  PIDController slideController = new PIDController(SLIDE_P, SLIDE_I, SLIDE_D, 0.018);
    protected final FtcDashboard dashboard = FtcDashboard.getInstance();
    // Slide motor right has the slide encoder
    protected DcMotorEx frontLeft, frontRight, rearLeft, rearRight,
            armPitMotor, slideMotorLeft, slideMotorRight, slideMotorOut;
    protected CRServo intakeLeft, intakeRight;
    protected RevColorSensorV3 intakeSensor;
    protected IMU imu;
    protected Controller controller1, controller2;
    protected TelemetryPacket packet = new TelemetryPacket();
    private double prevArmP = ARM_P, prevArmI = ARM_I, prevArmD = ARM_D;
    private double prevSlideP = SLIDE_P, prevSlideI = SLIDE_I, prevSlideD = SLIDE_D;

    ElapsedTimer timer;


    /**
     * Initializes all hardware components and their configurations.
     * Sets up motor directions, modes, and zero power behaviors.
     * Configures the IMU with the orientation parameters.
     */
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeftDrive");
        rearRight = hardwareMap.get(DcMotorEx.class, "RearRightDrive");

        armPitMotor = hardwareMap.get(DcMotorEx.class, "ArmMotorRight");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "SlideMotorLeft");
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "SlideMotorRight");
        slideMotorOut = hardwareMap.get(DcMotorEx.class, "SlideMotorOut");

        intakeLeft = hardwareMap.get(CRServo.class, "LeftIntakeCRServo");
        intakeRight = hardwareMap.get(CRServo.class, "RightIntakeCRServo");

        intakeSensor = hardwareMap.get(RevColorSensorV3.class, "IntakeColorSensor");

        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        armPitMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorOut.setDirection(DcMotorSimple.Direction.REVERSE);

        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorOut.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPitMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        //Initialize IMU if needed
        imu = hardwareMap.get(IMU.class, "IMU 1");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        timer = new ElapsedTimer();
    }

    /**
     * Periodic update during the `init` phase of the OpMode.
     * Ensures PID controller values are updated dynamically and gamepad input is processed.
     */
    @Override
    public void init_loop() {
        super.init_loop();
        checkPIDControllers();

        controller1.update();
        controller2.update();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        timer.updatePeriodTime();
        displayData("Loop Time", timer.getAveragePeriodSec());
    }

    @Override
    public void start() {
        super.start();
        timer.clearPastPeriods();
    }

    /**
     * Main loop for controlling the robot.
     * Updates PID controllers, processes controller input, and updates telemetry data.
     */
    @Override
    public void loop() {
        checkPIDControllers();

        controller1.update();
        controller2.update();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        timer.updatePeriodTime();
        displayData("Loop Time", timer.getAveragePeriodSec());
    }

    /**
     * Ensures all motors and hardware are stopped and reset at the end of the OpMode.
     */
    @Override
    public void stop() {
        super.stop();
    }

    /**
     * Dynamically checks if the PID controller parameters have changed
     * and updates the controllers accordingly to avoid inconsistencies.
     */
    private void checkPIDControllers() {
        // Check if the arm PID constants were updated; if so, then update the PID controller
        if (prevArmP != ARM_P || prevArmI != ARM_I || prevArmD != ARM_D) {
            armController.setPID(ARM_P, ARM_I, ARM_D);
            armController.reset();
            prevArmP = ARM_P;
            prevArmI = ARM_I;
            prevArmD = ARM_D;
        }

        // Check if the slide PID constants were updated; if so, then update the PID controller
        if (prevSlideP != SLIDE_P || prevSlideI != SLIDE_I || prevSlideD != SLIDE_D) {
            slideController.setPID(SLIDE_P, SLIDE_I, SLIDE_D);
            slideController.reset();
            prevSlideP = SLIDE_P;
            prevSlideI = SLIDE_I;
            prevSlideD = SLIDE_D;
        }
    }

    /**
     * Sets power for both intake servos to control material intake.
     *
     * @param power Power value for the intake system (-1.0 to 1.0).
     */
    protected void setIntakePower(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    /**
     * Sets power for all slide motors to control the slide mechanism.
     *
     * @param power Power value for the slide motors (-1.0 to 1.0).
     */
    protected void setSlidePower(double power) {
        slideMotorOut.setPower(power);
        slideMotorLeft.setPower(power);
        slideMotorRight.setPower(power);
    }

    /**
     * Returns the current encoder position of the slide motor with the encoder.
     *
     * @return Slide encoder position in ticks.
     */
    public double getSlideEncoderPosition() {
        return slideMotorRight.getCurrentPosition();
    }

    /**
     * Calculates the maximum allowable slide extension based on ticks per rotation.
     *
     * @return Maximum extension limit in ticks.
     */
    public double getExtensionLimitTicks() {
        return (SLIDE_TICKS_PER_ROTATION * SLIDE_HORIZONTAL_LIMIT_ROTATIONS);
    }

    /**
     * Displays data on both the Driver Station telemetry and the FTC Dashboard.
     *
     * @param caption Caption or label for the data.
     * @param data    Data value to display.
     */
    public void displayData(String caption, Object data) {
        telemetry.addData(caption, data);
        packet.put(caption, data);
    }

    /**
     * Calculates feedforward power for the arm based on its angle and slide extension.
     * Includes optional feedforward cutoff adjustments for extreme angles.
     *
     * @return Feedforward power value.
     */
    protected double calculateArmFeedforward() {
        double armPosition = armPitMotor.getCurrentPosition();
        double positionOffset = armPosition + ARM_ZERO_OFFSET;
        double armAngle = positionOffset / ARM_TICKS_PER_DEGREE;
        double armFeedforwardDifference = ARM_FEEDFORWARD_MAX - ARM_FEEDFORWARD;

        double percentOfExtension = Math.pow(getSlideEncoderPosition() / (getExtensionLimitTicks()), ARM_FEEDFORWARD_EXPONENT);

        // Calculate feedforward power based on arm angle and slide extension
        double kg = ARM_FEEDFORWARD * Math.cos((Math.toRadians(armAngle)));
        double ff = kg + (armFeedforwardDifference * percentOfExtension);

        // Adjust feedforward for extreme angles if cutoff is enabled
        if (useFeedforwardCutoff) {
            if (armPosition < ARM_LOW_ANGLE_FEEDFORWARD_CUTOFF * ARM_TICKS_PER_DEGREE) {
                ff = 0;
            } else if (armPosition > ARM_HIGH_ANGLE_FEEDFORWARD_CUTOFF * ARM_TICKS_PER_DEGREE) { // reduce FF when near or at vertical
                armFeedforwardDifference = armFeedforwardDifference - 0.2; //ToDo Make the reduction a constant
                ff = kg + (armFeedforwardDifference * percentOfExtension);
            }
        }
        return ff;
    }

    /**
     * Calculates the total power required for the arm, combining PID output and feedforward.
     *
     * @param setpoint Target position for the arm in ticks.
     * @return Total arm power value.
     */
    protected double calculateArmPower(double setpoint) {
        return armController.calculate(armPitMotor.getCurrentPosition(), setpoint) + calculateArmFeedforward();
    }

    /**
     * Calculates feedforward power for the slide mechanism based on arm angle and slide extension.
     * Ensures feedforward is zero if the slides are fully retracted.
     *
     * @return Feedforward power value for the slide mechanism.
     */
    protected double calculateSlideFeedforward() {
        double armPosition = armPitMotor.getCurrentPosition();
        double positionOffset = armPosition + ARM_ZERO_OFFSET;
        double armAngle = positionOffset / ARM_TICKS_PER_DEGREE;

        double slidePercentOfExtension = Math.pow(getSlideEncoderPosition() / (getExtensionLimitTicks()), SLIDE_FEEDFORWARD_EXPONENT);

        // Remove feedforward if the slides are fully retracted
        if (getSlideEncoderPosition() > 0) {
            slidePercentOfExtension = 0.0;
        }

        double slideFeedforwardDifference = SLIDE_FEEDFORWARD_MAX - SLIDE_FEEDFORWARD_BASE;
        double slideKs = slidePercentOfExtension * slideFeedforwardDifference + SLIDE_FEEDFORWARD_BASE;

        double slideFeedforward = slideKs * Math.sin(Math.toRadians(armAngle));

        if (Double.isNaN(slideFeedforward)) {
            slideFeedforward = 0;
        }

        return slideFeedforward;
    }
}