package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class ArmTuner extends RobotHardware {

    public static double setpoint = 0.0;

    public static double armP = 0.01;
    public static double armI = 0.0;
    public static double armD = 0.0;

    public static double armLowerLimit = -167;
    public static double armUpperLimit = 980;

    public static double zeroOffset = 100;
    public static double ticksPerDegree = 10;
    public static double ff = 0.1;

    public static boolean feedforward = false;

    private double prevP = armP, prevI = armI, prevD = armD;

    @Override
    public void loop() {
        super.loop();

        if (prevP != armP || prevI != armI || prevD != armD) {
            armController.setPID(armP, armI, armD);
            armController.reset();

            prevP = armP;
            prevI = armI;
            prevD = armD;
        }
        setpoint = Math.min(armUpperLimit, Math.max(armLowerLimit, setpoint));

        double position = armPitMotor.getCurrentPosition();
        double power = armController.calculate(position, setpoint);
        telemetry.addData("Power", power);
        if (feedforward) {
            double kg = ff * Math.cos((position + zeroOffset)/ticksPerDegree);
            power += kg;
            telemetry.addData("KG", kg);
            armPitMotor.setPower(kg);
        } else {
            telemetry.addData("Compensated Power", power);
            armPitMotor.setPower(power);
        }
    }
}
