package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class AutonV1 extends LinearOpMode{
    DistanceSensor FD, LD, RD, BD;
    DcMotor BL, FL, FR, BR;
    double yA, xA, ticksPerInch, divisor, xTick, yTick, lM, rM;
    public void allM_Mode(DcMotor.RunMode command){BL.setMode(command); FL.setMode(command); FR.setMode(command); BR.setMode(command);}
    public void allM_Power(double power){BL.setPower(power); FL.setPower(power); FR.setPower(power); BR.setPower(power);}
    private ElapsedTime runtime = new ElapsedTime();
    public boolean BL_IsBusy(double x, double y){
        if (Math.abs(BL.getCurrentPosition()) < Math.abs(y + x) * ticksPerInch){
            return true;
        }
        else{
            return false;
        }
    }
    public void driveToCoords(double x, double y, double speed){
        allM_Mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        allM_Mode(DcMotor.RunMode.RUN_USING_ENCODER);
        divisor = Math.max(Math.abs(x), Math.abs(y));
        yA = y / divisor;
        xA = x / -divisor;
        lM = yA + xA;
        rM = yA - xA;
        BL.setPower(lM);
        FL.setPower(rM);
        FR.setPower(lM);
        BR.setPower(rM);
        System.out.println(BL.getPower());
        System.out.println(FL.getPower());
        System.out.println(FR.getPower());
        System.out.println(BR.getPower());
        runtime.reset();
        System.out.println(Math.abs(y + x) * ticksPerInch);
        while (!isStopRequested() && BL_IsBusy(x, y)){
        }
        allM_Power(0);
    }
    public void getCoordsEncTicks(){
        xTick = (-BL.getCurrentPosition() + FL.getCurrentPosition() - FR.getCurrentPosition() + BR.getCurrentPosition()) / ticksPerInch;
    }
    @Override
    public void runOpMode(){
        BL = hardwareMap.dcMotor.get("back_left_motor");
        FL = hardwareMap.dcMotor.get("front_left_motor");
        FR = hardwareMap.dcMotor.get("front_right_motor");
        BR = hardwareMap.dcMotor.get("back_right_motor");
        FD = hardwareMap.get(DistanceSensor.class, "front_distance");
        LD = hardwareMap.get(DistanceSensor.class, "left_distance");
        RD = hardwareMap.get(DistanceSensor.class, "right_distance");
        BD = hardwareMap.get(DistanceSensor.class, "back_distance");
        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);
        FR.setDirection(FORWARD);
        BR.setDirection(FORWARD);
        runtime.reset();
        allM_Mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        allM_Mode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setPower(1);
        FL.setPower(1);
        FR.setPower(1);
        BR.setPower(1);
        while (FD.getDistance(DistanceUnit.INCH) > 2 && !isStopRequested()){}
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        ticksPerInch = BL.getCurrentPosition() / 70;
        System.out.println(ticksPerInch);
        driveToCoords(40, -100, 1);
        while (!isStopRequested()){
        }
    }
}
