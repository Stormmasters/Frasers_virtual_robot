package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class AutonBasic extends LinearOpMode {
    DistanceSensor LD, RD, FD, BD;
    DcMotor BL, FL, FR, BR;
    double ticksPerInch, initDistance, x, y;
    public void allM_Mode(DcMotor.RunMode command){BL.setMode(command); FL.setMode(command); FR.setMode(command); BR.setMode(command);}
    public void allM_Power(double power){BL.setPower(power); FL.setPower(power); FR.setPower(power); BR.setPower(power);}
    public void getTicksPerInch(){
        allM_Mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        allM_Mode(DcMotor.RunMode.RUN_USING_ENCODER);
        allM_Power(1);
        initDistance = FD.getDistance(DistanceUnit.INCH);
        System.out.println(initDistance);
        while (FD.getDistance(DistanceUnit.INCH) > initDistance - 2){
            System.out.println(FD.getDistance(DistanceUnit.INCH));
        }
        allM_Power(0);
        ticksPerInch = (double) BL.getCurrentPosition() / 2;
        System.out.println(ticksPerInch);
    }
    public void getPosition(){
        y = FD.getDistance(DistanceUnit.INCH);
        x = LD.getDistance(DistanceUnit.INCH);
        System.out.println(x);
        if (x > 72){
            x = -x + 72;
        }
        if (y > 72){
            y = -y + 72;
        }
        System.out.println(x + ", " + y);
    }
    @Override
    public void runOpMode(){BL = hardwareMap.dcMotor.get("back_left_motor");
        FL = hardwareMap.dcMotor.get("front_left_motor");
        FR = hardwareMap.dcMotor.get("front_right_motor");
        BR = hardwareMap.dcMotor.get("back_right_motor");
        LD = hardwareMap.get(DistanceSensor.class, "left_distance");
        RD = hardwareMap.get(DistanceSensor.class, "right_distance");
        FD = hardwareMap.get(DistanceSensor.class, "front_distance");
        BD = hardwareMap.get(DistanceSensor.class, "back_distance");
        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);
        FR.setDirection(FORWARD);
        BR.setDirection(FORWARD);
        getPosition();
        while (opModeIsActive()){
        }
    }
}
