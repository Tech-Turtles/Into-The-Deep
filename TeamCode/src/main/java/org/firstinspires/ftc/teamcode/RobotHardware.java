package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utility.Controller;

public class RobotHardware extends OpMode {
    protected DcMotorEx frontLeft, frontRight, rearLeft, rearRight;
    protected CRServo intakeLeft, intakeRight;
    protected IMU imu;
    protected Controller controller1, controller2;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeftDrive");
        rearRight = hardwareMap.get(DcMotorEx.class, "RearRightDrive");

        intakeLeft = hardwareMap.get(CRServo.class, "LeftIntakeCRServo");
        intakeRight = hardwareMap.get(CRServo.class, "RightIntakeCRServo");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

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