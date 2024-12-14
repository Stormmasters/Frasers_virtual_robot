package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorTest extends LinearOpMode {
    DcMotor BL, FL, FR, BR;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        BL = hardwareMap.dcMotor.get("back_left_motor");
        FL = hardwareMap.dcMotor.get("front_left_motor");
        FR = hardwareMap.dcMotor.get("front_right_motor");
        BR = hardwareMap.dcMotor.get("back_right_motor");
        while (!isStopRequested()){
            runtime.reset();
            while (runtime.seconds() < 2){
                FR.setPower(1);
            }
            FR.setPower(0);
            while (runtime.seconds() < 4){
                FL.setPower(1);
            }
            FL.setPower(0);
            while (runtime.seconds() < 6){
                BR.setPower(1);
            }
            BR.setPower(0);
            while (runtime.seconds() < 8){
                BL.setPower(1);
            }
            BL.setPower(0);
        }
    }
}
