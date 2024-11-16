package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FraserSimpleAutonV1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor BL, FL, FR, BR;
    @Override
    public void runOpMode(){
        BL = hardwareMap.dcMotor.get("back_left_motor");
        FL = hardwareMap.dcMotor.get("front_left_motor");
        FR = hardwareMap.dcMotor.get("front_right_motor");
        BR = hardwareMap.dcMotor.get("back_right_motor");
        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);
        FR.setDirection(FORWARD);
        BR.setDirection(FORWARD);
        BL.setPower(0.5);
        FL.setPower(0.5);
        FR.setPower(0.5);
        BR.setPower(0.5);
        while (runtime.seconds() < 1){}
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
}
