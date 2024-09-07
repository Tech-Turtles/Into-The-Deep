package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Manual", group = "A")
//@Disabled
public class Manual extends RobotHardware {

    // Max speed in slow mode set to 40%
    private final double slowModeMultiplier = 0.4;
    // Flag to control whether slow mode is on or not
    private boolean slowModeEnabled = false;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
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
    }
}