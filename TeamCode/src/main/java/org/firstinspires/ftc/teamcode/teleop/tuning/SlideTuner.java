package org.firstinspires.ftc.teamcode.teleop.tuning;

import static org.firstinspires.ftc.teamcode.Constants.ARM_BUCKET_SAMPLE_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_FEEDFORWARD;
import static org.firstinspires.ftc.teamcode.Constants.ARM_FEEDFORWARD_EXPONENT;
import static org.firstinspires.ftc.teamcode.Constants.ARM_FEEDFORWARD_MAX;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_ANGLE_FEEDFORWARD_CUTOFF;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_CHAMBER_END_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_SAMPLE_PIVOT_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH_SPEC_PIVOT_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_HORIZONTAL_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_LOW_ANGLE_FEEDFORWARD_CUTOFF;
import static org.firstinspires.ftc.teamcode.Constants.ARM_LOW_SAMPLE_PIVOT_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_VERTICAL_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ARM_ZERO_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.HIGH_SAMPLE_EXT_SLIDE;
import static org.firstinspires.ftc.teamcode.Constants.HIGH_SPEC_EXT_SLIDE;
import static org.firstinspires.ftc.teamcode.Constants.LOW_SAMPLE_EXT_SLIDE;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_FEEDFORWARD_BASE;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_FEEDFORWARD_EXPONENT;
import static org.firstinspires.ftc.teamcode.Constants.SLIDE_FEEDFORWARD_MAX;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * SlideTuner is a TeleOp tuning class for a rotating arm and linear slides.
 * It allows tuning of feedforward and PID parameters for precise control.
 */
@Config
@TeleOp(group = "B")
public class SlideTuner extends RobotHardware {
    public static double maxSlideSpeed = 1.0;
    /**
     * Target position for the arm PID controller
     */
    public static double armSetpoint = 0;
    /**
     * Target position for the slide PID controller
     */
    public static double slideSetpoint = 0;
    public static boolean verbose = false;
    public static boolean enableArm = true;
    public static TuningMode tuningMode = TuningMode.OFF;

    @Override
    public void loop() {
        super.loop();

        telemetry.addLine(String.format(
                "<h1 style='color:yellow;'>%s</h1>",
                tuningMode.name()
        ));

        if (controller2.dpadLeft()) {
            armSetpoint = ARM_BUCKET_SAMPLE_ANGLE;
        }

        if (controller2.dpadUp()) {
            armSetpoint = ARM_VERTICAL_POSITION;
        }

        if (controller2.dpadDown()) {
            armSetpoint = ARM_HORIZONTAL_POSITION;
        }

        if (controller2.dpadRight()) {
            armSetpoint = ARM_HIGH_CHAMBER_END_POSITION;
        }

        if (controller2.triangle()) {
            slideSetpoint = HIGH_SPEC_EXT_SLIDE;
            armSetpoint = ARM_HIGH_SPEC_PIVOT_ANGLE;
        }

        if (controller2.cross()) {
            slideSetpoint = HIGH_SAMPLE_EXT_SLIDE;
            armSetpoint = ARM_HIGH_SAMPLE_PIVOT_ANGLE;
        }

        if (controller2.square()) {
            slideSetpoint = LOW_SAMPLE_EXT_SLIDE;
            armSetpoint = ARM_LOW_SAMPLE_PIVOT_ANGLE;
        }

        double armPosition = armPitMotor.getCurrentPosition();
        double positionOffset = armPosition + ARM_ZERO_OFFSET;
        double armAngle = positionOffset / ARM_TICKS_PER_DEGREE;

        displayData("Arm Angle", armAngle);
        if (verbose) {
            displayData("Arm Position", armPosition);
            displayData("Arm Compensated Position", positionOffset);
        }

        if (enableArm) {
            double armPower = 0;
            double armFeedforwardDifference = ARM_FEEDFORWARD_MAX - ARM_FEEDFORWARD;

            double percentOfExtension = Math.pow(getSlideEncoderPosition() / (getExtensionLimitTicks()), ARM_FEEDFORWARD_EXPONENT);

            if (verbose) {
                displayData("Arm Feedforward Difference", armFeedforwardDifference);
                displayData("Arm Slide Percent of Extension", percentOfExtension);
            }

            // Calculate feedforward power based on arm angle and slide extension
            double kg = ARM_FEEDFORWARD * Math.cos((Math.toRadians(armAngle)));
            double ff = kg + (armFeedforwardDifference * percentOfExtension);
            displayData("Arm FF", ff);

            // Adjust feedforward for extreme angles if cutoff is enabled
            if (useFeedforwardCutoff) {
                if (armPosition < ARM_LOW_ANGLE_FEEDFORWARD_CUTOFF * ARM_TICKS_PER_DEGREE) {
                    ff = 0;
                } else if (armPosition > ARM_HIGH_ANGLE_FEEDFORWARD_CUTOFF * ARM_TICKS_PER_DEGREE) {
                    armFeedforwardDifference = armFeedforwardDifference - 0.2;
                    ff = kg + (armFeedforwardDifference * percentOfExtension);
                }
                displayData("Compensated Arm FF", ff);
            }
            armPower += ff;

            double pidPower = armController.calculate(armPitMotor.getCurrentPosition(), armSetpoint);
            displayData("Arm PID Power", pidPower);
            displayData("Arm PID Setpoint", armSetpoint);
            // Add PID control power
            armPower += pidPower;

            displayData("Commanded Total Arm Power", armPower);

            armPitMotor.setPower(armPower);
        } else { // Disable the arm motor if arm control is not enabled
            armPitMotor.setPower(0);
        }

        double slideFeedforward = 0;
        double slidePID = 0;

        double slidePercentOfExtension = Math.pow(getSlideEncoderPosition() / (getExtensionLimitTicks()), SLIDE_FEEDFORWARD_EXPONENT);
        if (verbose)
            displayData("Slide Percent Extension", slidePercentOfExtension);
        // Remove feedforward if the slides are fully retracted
        if (getSlideEncoderPosition() > 0) {
            slidePercentOfExtension = 0.0;
            displayData("Slide Percent Extension Compensated", slidePercentOfExtension);
        }
        double slideFeedforwardDifference = SLIDE_FEEDFORWARD_MAX - SLIDE_FEEDFORWARD_BASE;
        double slideKs = slidePercentOfExtension * slideFeedforwardDifference + SLIDE_FEEDFORWARD_BASE;
        double slideFeedforwardCalc = slideKs * Math.sin(Math.toRadians(armAngle));
        if (Double.isNaN(slideFeedforwardCalc)) {
            slideFeedforwardCalc = 0;
            displayData("Slide Percent Extension NaN", slidePercentOfExtension);
        }

        // Display detailed slide feedforward and position data for tuning
        if (verbose) {
            displayData("Slide Feedforward Difference", slideFeedforwardDifference);
            displayData("Slide Ks", slideKs);
        }
        displayData("Slide Feedforward", slideFeedforwardCalc);

        // Use feedforward tuning if enabled
        if (tuningMode.equals(TuningMode.FEEDFORWARD) || tuningMode.equals(TuningMode.BOTH)) {
            slideFeedforward = slideFeedforwardCalc;
        }

        double slidePIDCalc = slideController.calculate(getSlideEncoderPosition(), slideSetpoint);
        displayData("Slide PID Power", slidePIDCalc);
        displayData("Slide Setpoint", slideSetpoint);
        displayData("Slide Position", getSlideEncoderPosition());
        if (tuningMode.equals(TuningMode.PID) || tuningMode.equals(TuningMode.BOTH)) {
            slidePID = slidePIDCalc;
        }

        // Manual slide control using triggers has priority over automatic control
        if (controller2.right_trigger > 0.2) {
            setSlidePower(controller2.right_trigger * maxSlideSpeed - slideFeedforward);
        } else if (controller2.left_trigger > 0.2) {
            setSlidePower(controller2.left_trigger * -maxSlideSpeed - slideFeedforward);
        } else if (controller2.triangle() || controller2.square() || controller2.cross()) {
            setSlidePower(-(slidePID + slideFeedforward));
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