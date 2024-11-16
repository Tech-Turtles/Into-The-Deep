package org.firstinspires.ftc.teamcode;
//automatically imports code
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Cheeseburger")
public class JavaTest extends OpMode {
    boolean intDone;
    int x;
    int y;

    public JavaTest(int x, int y){
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString(){
        return "JavaTest " + x + " " + y;
    }

    @Override
    public void init() {
        telemetry.addData("int Done", intDone);
        intDone = true;
    }
    //returnDataType name(parameters)
    double squareInputWithSign(double input){
        double output = input * input;
        if (input < 0) {
            output = output * -1;
        }
        return output;
    }

    @Override
    public void loop() {
        double leftAmount = gamepad1.left_stick_x;
        double fwdAmount = -gamepad1.left_stick_y;

        telemetry.addData("Before X", leftAmount);
        telemetry.addData("Before Y", fwdAmount);

        leftAmount = squareInputWithSign(leftAmount);
        fwdAmount = squareInputWithSign(fwdAmount);

        telemetry.addData("After X", leftAmount);
        telemetry.addData("After Y", fwdAmount);

        telemetry.addData("int Done", intDone);

    }
}
