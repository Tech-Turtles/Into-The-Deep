package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.Controller;

@TeleOp(name = "Mecanum Test", group = "B")
@Disabled
public class MecanumTest extends OpMode {
    protected DcMotor frontLeft, frontRight, rearLeft, rearRight;
    protected IMU imu;
    protected Controller controller1, controller2;


    // Max speed in slow mode set to 40%
    private final double slowModeMultiplier = 0.4;
    // Flag to control whether slow mode is on or not
    private boolean slowModeEnabled = false;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        rearLeft = hardwareMap.get(DcMotor.class, "RearLeftDrive");
        rearRight = hardwareMap.get(DcMotor.class, "RearRightDrive");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

         //Initialize IMU if needed
         imu = hardwareMap.get(IMU.class, "IMU 1");
         IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                 RevHubOrientationOnRobot.LogoFacingDirection.UP,
                 RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
         imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        controller1.update();
        controller2.update();
    }

    @Override
    public void loop() {
        controller1.update();
        controller2.update();

        // Reset gyro angle if triangle is pressed
        if (controller1.triangleOnce()){
            imu.resetYaw();
        }

        // Toggle slow mode on or off if cross is pressed
        if (controller1.crossOnce()) {
            slowModeEnabled = !slowModeEnabled;
        }

        double y = -controller1.left_stick_y; // Remember, Y stick value is reversed
        double x = controller1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = controller1.right_stick_x;

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
    }
}