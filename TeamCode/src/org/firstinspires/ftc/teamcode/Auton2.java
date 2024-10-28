package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Auton2 extends LinearOpMode {
    DistanceSensor FD;
    DcMotor BL, FL, FR, BR;
    double xM, yM, m = 1, xP, yP;
    boolean precision = false;
    public double BLp(){return (BL.getCurrentPosition() / 78.2 / 4);}
    public double FLp(){return (FL.getCurrentPosition() / 78.2 / 4);}
    public double FRp(){return (FR.getCurrentPosition() / 78.2 / 4);}
    public double BRp(){return (BR.getCurrentPosition() / 78.2 / 4);}
    public double xPos(){return -BLp() + FLp() - FRp() + BRp();}
    public double yPos(){return BLp() + FLp() + FRp() + BRp();}
    public double xDif(double x){return Math.abs(x) - Math.abs(xPos());}
    public double yDif(double y){return Math.abs(y) - Math.abs(yPos());}
    public void allM_Mode(DcMotor.RunMode command){BL.setMode(command); FL.setMode(command); FR.setMode(command); BR.setMode(command);}
    public void allM_Power(double power){BL.setPower(power); FL.setPower(power); FR.setPower(power); BR.setPower(power);}
    private ElapsedTime     runtime = new ElapsedTime();
    private DistanceSensor distanceSensor;
    public void crawlToCoords(double x, double y){
        m = 1;
        while (opModeIsActive()){
            System.out.println("xDif, yDif(): " + xDif(x) + ", " + yDif(y));
            xP = x - xPos();
            yP = y - yPos();
            System.out.println("xP, yP: " + xP + ", " + yP);
            xM = xP * m / Math.max(x, y);
            yM = yP * m / Math.max(x, y);
            BL.setPower(yM - xM);
            FL.setPower(yM + xM);
            FR.setPower(yM - xM);
            BR.setPower(yM + xM);
            if (xDif(x) < 0.001 || yDif(y) < 0.){
                System.out.println("hi");
                if (yDif(y) == 0 && Math.abs(xDif(x)) > 0){
                    BL.setPower(-xM * 200000000 * xDif(x));
                    FL.setPower(xM * 200000000 * xDif(x));
                    FR.setPower(-xM * 200000000 * xDif(x));
                    BR.setPower(xM * 200000000 * xDif(x));
                }
                else if (xDif(x) == 0 && Math.abs(yDif(y)) > 0){
                    allM_Power(yM * 200000000 * yDif(y));
                }
                else if (xDif(x) == 0 && yDif(y) == 0) {
                    allM_Power(0);
                    break;
                }
                if (Math.abs(xDif(x)) < 0.01 && Math.abs(yDif(y)) < 0.01){
                    allM_Power(0);
                    break;
                }
            }
        }
    }
    @Override
    public void runOpMode(){
        System.out.println(opModeIsActive());
        BL = hardwareMap.dcMotor.get("back_left_motor");
        FL = hardwareMap.dcMotor.get("front_left_motor");
        FR = hardwareMap.dcMotor.get("front_right_motor");
        BR = hardwareMap.dcMotor.get("back_right_motor");
        FD = hardwareMap.get(DistanceSensor.class, "front_distance");
        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);
        FR.setDirection(FORWARD);
        BR.setDirection(FORWARD);
        waitForStart();
        runtime.reset();
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        System.out.println("Done");
        while (opModeIsActive()){
            crawlToCoords(30, -50);
            crawlToCoords(30, 50);
            crawlToCoords(0,0);
        }
    }
}
