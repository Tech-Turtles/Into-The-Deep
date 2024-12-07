package org.firstinspires.ftc.teamcode.teleop.tuning;

import static org.firstinspires.ftc.teamcode.Constants.ARM_FEEDFORWARD;
import static org.firstinspires.ftc.teamcode.Constants.ARM_FEEDFORWARD_EXPONENT;
import static org.firstinspires.ftc.teamcode.Constants.ARM_FEEDFORWARD_MAX;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_ANGLE_FEEDFORWARD_CUTOFF;
import static org.firstinspires.ftc.teamcode.Constants.ARM_LOW_ANGLE_FEEDFORWARD_CUTOFF;
import static org.firstinspires.ftc.teamcode.Constants.ARM_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_ZERO_OFFSET;
import static org.firstinspires.ftc.teamcode.teleop.MainPID.maxSlideSpeed;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp(group = "B")
public class ArmTuner extends RobotHardware {
    public static double setpoint = 0; // Ticks
    public static boolean verbose = false;
    public static TuningMode tuningMode = TuningMode.OFF;

    @Override
    public void loop() {
        super.loop();

        telemetry.addLine(String.format(
                "<h1 style='color:yellow;'>%s</h1>",
                tuningMode.name()
        ));

        double armPower = 0;

        double armPosition = armPitMotor.getCurrentPosition();
        double positionOffset = armPosition + ARM_ZERO_OFFSET;
        double armAngle = positionOffset / ARM_TICKS_PER_DEGREE;
        double ffDifference = ARM_FEEDFORWARD_MAX - ARM_FEEDFORWARD;

        double percentOfExtension = Math.pow(getSlideEncoderPosition() / (getExtensionLimitTicks()), ARM_FEEDFORWARD_EXPONENT);

        if (verbose) {
            displayData("Arm Position", armPosition);
            displayData("Arm Feedforward Difference", ffDifference);
            displayData("Arm Compensated Position", positionOffset);
            displayData("Arm Angle", armAngle);
            displayData("Slide Position", getSlideEncoderPosition());
            displayData("Slide Percent of Extension", percentOfExtension);
        }

        double kg = ARM_FEEDFORWARD * Math.cos((Math.toRadians(armAngle)));
        double ff = kg + (ffDifference * percentOfExtension);
        displayData("Arm FF", ff);

        if (tuningMode.equals(TuningMode.FEEDFORWARD) || tuningMode.equals(TuningMode.BOTH)) {
            if (useFeedforwardCutoff) {
                if (armPosition < ARM_LOW_ANGLE_FEEDFORWARD_CUTOFF * ARM_TICKS_PER_DEGREE) {
                    ff = 0;
                } else if (armPosition > ARM_HIGH_ANGLE_FEEDFORWARD_CUTOFF * ARM_TICKS_PER_DEGREE) {
                    ffDifference = ffDifference - 0.2;
                    ff = kg + (ffDifference * percentOfExtension);
                }
                displayData("Compensated Arm FF", ff);
            }
            armPower += ff;
        }

        double pidPower = armController.calculate(armPitMotor.getCurrentPosition(), setpoint);
        displayData("Arm PID Power", pidPower);
        displayData("Arm PID Setpoint", setpoint);

        if (tuningMode.equals(TuningMode.PID) || tuningMode.equals(TuningMode.BOTH)) {
            armPower += pidPower;
        }

        displayData("Calculated Total Arm Power", armPower);

        if (tuningMode.equals(TuningMode.OFF)) {
            armPower = 0;
        }

        displayData("Commanded Total Arm Power", armPower);

        armPitMotor.setPower(armPower);

        // Manual slide controls
        if (controller1.right_trigger > 0.2) {
            if (!controller1.circle() && Math.abs(slideMotorRight.getCurrentPosition()) > getExtensionLimitTicks()) {
                setSlidePower(0);
            } else {
                setSlidePower(controller1.right_trigger * maxSlideSpeed);
            }
        } else if (controller1.left_trigger > 0.2) {
            setSlidePower(controller1.left_trigger * -maxSlideSpeed);
        } else {
            setSlidePower(0);
        }
    }

    public enum TuningMode {
        OFF,          // No tuning or adjustments
        FEEDFORWARD,  // Apply feedforward adjustments only
        PID,          // Apply PID adjustments only
        BOTH          // Apply both feedforward and PID adjustments
    }
}