package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utility.Controller;
import org.firstinspires.ftc.teamcode.utility.PIDController;

public class RobotHardware extends OpMode {
    protected DcMotorEx frontLeft, frontRight, rearLeft, rearRight,
            armMotorRight, slideMotorLeft, slideMotorRight, slideMotorOut;
    protected CRServo intakeLeft, intakeRight;
    protected RevColorSensorV3 intakeSensor;
    protected IMU imu;
    protected Controller controller1, controller2;

    protected final PIDController armController = new PIDController(0.01, 0.0, 0.0, 0.018);

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeftDrive");
        rearRight = hardwareMap.get(DcMotorEx.class, "RearRightDrive");

        armMotorRight = hardwareMap.get(DcMotorEx.class, "ArmMotorRight");
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "SlideMotorLeft");
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "SlideMotorRight");
        slideMotorOut = hardwareMap.get(DcMotorEx.class, "SlideMotorOut");

        intakeLeft = hardwareMap.get(CRServo.class, "LeftIntakeCRServo");
        intakeRight = hardwareMap.get(CRServo.class, "RightIntakeCRServo");

        intakeSensor = hardwareMap.get(RevColorSensorV3.class, "IntakeColorSensor");

        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorOut.setDirection(DcMotorSimple.Direction.REVERSE);

        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorOut.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    }

    @Override
    public void init_loop() {
        super.init_loop();
        controller1.update();
        controller2.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        controller1.update();
        controller2.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}