package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class NEW extends LinearOpMode{
    DistanceSensor FD, LD, RD, BD;
    DcMotor BL, FL, FR, BR;
    double ticksPerInch, x = 0, y = 0, FDy, BDy, LDx, RDx, distanceAdjusted, speed;
    private ElapsedTime     runtime = new ElapsedTime();
    public void strafe(double distance, double speed, boolean adjust){
        updateCoords();
        System.out.println(x + ", " + y);
        distanceAdjusted = distance - x;
        allM_Reset();
        allM_Encoder();
        BL.setPower(-distanceAdjusted / Math.abs(distanceAdjusted) * speed);
        FL.setPower(distanceAdjusted / Math.abs(distanceAdjusted) * speed);
        FR.setPower(-distanceAdjusted / Math.abs(distanceAdjusted) * speed);
        BR.setPower(distanceAdjusted / Math.abs(distanceAdjusted) * speed);
        while (Math.abs(BL.getCurrentPosition()) < Math.abs(distanceAdjusted) * ticksPerInch && !isStopRequested()){
            telemetry.addData("running... Progress: ", BL.getCurrentPosition());
        }
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 0.1 && isStopRequested()){}
        updateCoords();
        System.out.println(x + ", " + y);
        if (!adjust){errorCorrect(distance, "x");}
    }
    public void drive(double distance, double speed, boolean adjust){
        updateCoords();
        distanceAdjusted = distance - y;
        allM_Reset();
        allM_Encoder();
        BL.setPower(distanceAdjusted / Math.abs(distanceAdjusted) * speed);
        FL.setPower(distanceAdjusted / Math.abs(distanceAdjusted) * speed);
        FR.setPower(distanceAdjusted / Math.abs(distanceAdjusted) * speed);
        BR.setPower(distanceAdjusted / Math.abs(distanceAdjusted) * speed);
        while (Math.abs(BL.getCurrentPosition()) < Math.abs(distanceAdjusted) * ticksPerInch && !isStopRequested()){
            telemetry.addData("running... Progress: ", BL.getCurrentPosition());
        }
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        runtime.reset();
        while (runtime.seconds() < 0.1 && !isStopRequested()){}
        updateCoords();
        System.out.println(x + ", " + y);
        if (!adjust){errorCorrect(distance, "y");}
    }
    public void errorCorrect(double correction, String type){
        updateCoords();
        if (type.equals(x)){
            speed = Math.abs(x - correction);
            if (speed > 1){speed = 1;}
            strafe(x - correction, speed, true);
            System.out.println("hi");
        }
        if (type.equals(y)){
            speed = Math.abs(y - correction);
            if (speed > 1){speed = 1;}
            drive(y - correction, speed, true);
            System.out.println("hello");
        }
    }
    public void updateCoords(){
        FDy = FD.getDistance(DistanceUnit.INCH);
        BDy = BD.getDistance(DistanceUnit.INCH);
        LDx = LD.getDistance(DistanceUnit.INCH);
        RDx = RD.getDistance(DistanceUnit.INCH);
        if (FDy < 72 && RDx < 72){
            x = 72 - RDx;
            y = 72 - FDy;
            System.out.println("1");
        }
        else if (BDy < 72 && RDx < 72){
            x = 72 - RDx;
            y = -72 + BDy;
            System.out.println("2");
        }
        else if (FDy < 72 && LDx < 72){
            x = -72 + LDx;
            y = 72 - FDy;
            System.out.println("3");
        }
        else if (BDy < 72 && LDx < 72){
            x = -72 + LDx;
            y = -72 + BDy;
            System.out.println("4");
        }
    }
    public void allM_Reset(){
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void allM_Encoder(){
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        allM_Reset();
        allM_Encoder();
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
        System.out.println("Ticks per inch: " + ticksPerInch);
        runtime.reset();
        strafe(66, 1, false);
        while (runtime.seconds() < 0.5 && !isStopRequested()){}
        while (!isStopRequested()){
            strafe(66, 1, false);
            drive(-70, 1, false);
            strafe(-66, 1, false);
            drive(70, 1, false);
        }
    }
}