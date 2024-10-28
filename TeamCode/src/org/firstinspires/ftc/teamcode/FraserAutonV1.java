package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class FraserAutonV1 extends LinearOpMode {
    DistanceSensor FD, LD, RD, BD;
    DcMotor BL, FL, FR, BR;
    double initY, initX, x, y, m;
    boolean keepLoop = true;
    String yDistance, xDistance;
    public void allM_Power(double power){BL.setPower(power); FL.setPower(power); FR.setPower(power); BR.setPower(power);}
    public void allM_Mode(DcMotor.RunMode command){BL.setMode(command); FL.setMode(command); FR.setMode(command); BR.setMode(command);}
    private ElapsedTime runtime = new ElapsedTime();
    public void turn(double angle){
        BL.setPower(0.3 * angle / Math.abs(angle));
        FL.setPower(0.3 * angle / Math.abs(angle));
        FR.setPower(-0.3 * angle / Math.abs(angle));
        BR.setPower(-0.3 * angle / Math.abs(angle));
        while (!isStopRequested()){
            IMU imu = hardwareMap.get(IMU.class, "imu");
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (Math.abs(orientation.firstAngle) >= Math.abs(Math.toRadians(angle - 2))){
                BL.setPower(0.02 * angle / Math.abs(angle));
                FL.setPower(0.02 * angle / Math.abs(angle));
                FR.setPower(-0.02 * angle / Math.abs(angle));
                BR.setPower(-0.02 * angle / Math.abs(angle));
                if (Math.abs(orientation.firstAngle) >= Math.abs(Math.toRadians(angle - 0.005))){
                    break;
                }
            }
            System.out.println(orientation.firstAngle);
        }
        allM_Power(0);
    }
    public void moveToCoords(double X, double Y, boolean errorCorrect){
        if (FD.getDistance(DistanceUnit.INCH) < 60){
            initY = FD.getDistance(DistanceUnit.INCH);
            yDistance = "FD";
        }
        else{
            initY = BD.getDistance(DistanceUnit.INCH);
            yDistance = "BD";
        }
        if (LD.getDistance(DistanceUnit.INCH) < 60){
            initX = LD.getDistance(DistanceUnit.INCH);
            xDistance = "LD";
        }
        else{
            initX = RD.getDistance(DistanceUnit.INCH);
            xDistance = "RD";
        }
        x = 0.3 * X / Math.abs((Math.max(Math.abs(X), Math.abs(Y))));
        y = 0.3 * Y / Math.abs((Math.max(Math.abs(X), Math.abs(Y))));
        BL.setPower(y + x);
        FL.setPower(y - x);
        FR.setPower(y + x);
        BR.setPower(y - x);
        System.out.println("FD: " + FD.getDistance(DistanceUnit.INCH));
        System.out.println("LD: " + LD.getDistance(DistanceUnit.INCH));
        System.out.println("RD: " + RD.getDistance(DistanceUnit.INCH));
        System.out.println("BD: " + BD.getDistance(DistanceUnit.INCH));
        System.out.println("xDistance: " + xDistance);
        System.out.println("yDistance: " + yDistance);
        System.out.println("initX: " + initX);
        System.out.println("initY: " + initY);
        System.out.println("RD condition: " + (Math.abs(X) + initX));
        System.out.println("BD condition: " + (Math.abs(Y) + initY));
        System.out.println("hi");
        while (!isStopRequested()){
            System.out.println("FD: " + (FD.getDistance(DistanceUnit.INCH) - Math.abs(Y) + initY));
            System.out.println("LD: " + (LD.getDistance(DistanceUnit.INCH) - Math.abs(X) + initX));
            System.out.println("RD: " + (RD.getDistance(DistanceUnit.INCH) - Math.abs(X) + initX));
            System.out.println("BD: " + (BD.getDistance(DistanceUnit.INCH) - (Math.abs(Y) + initY)));
            System.out.println("xDistance: " + xDistance);
            System.out.println("yDistance: " + yDistance);
            if (xDistance.equals("LD") && (LD.getDistance(DistanceUnit.INCH) - (Math.abs(X) + initX)) > 0){
                break;
            }
            else if (xDistance.equals("RD") && (RD.getDistance(DistanceUnit.INCH) - (Math.abs(X) + initX)) > 0){
                break;
            }
            if (yDistance.equals("FD") && (FD.getDistance(DistanceUnit.INCH) - (Math.abs(Y) + initY)) > 0){
                break;
            }
            if (yDistance.equals("BD") && (BD.getDistance(DistanceUnit.INCH) - (Math.abs(Y) + initY)) > 0){
                break;
            }
            if (yDistance.equals("BD")){
                System.out.println("it works");
            }
        }
        System.out.println("hi");
        allM_Power(0);
    }
    @Override
    public void runOpMode(){
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        imu.resetYaw();
        BL = hardwareMap.dcMotor.get("back_left_motor");
        FL = hardwareMap.dcMotor.get("front_left_motor");
        FR = hardwareMap.dcMotor.get("front_right_motor");
        BR = hardwareMap.dcMotor.get("back_right_motor");
        FD = hardwareMap.get(DistanceSensor.class, "front_distance");
        LD = hardwareMap.get(DistanceSensor.class, "left_distance");
        RD = hardwareMap.get(DistanceSensor.class, "right_distance");
        BD = hardwareMap.get(DistanceSensor.class, "back_distance");
        FD.getDistance(DistanceUnit.INCH);
        LD.getDistance(DistanceUnit.INCH);
        RD.getDistance(DistanceUnit.INCH);
        BD.getDistance(DistanceUnit.INCH);
        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);
        FR.setDirection(FORWARD);
        BR.setDirection(FORWARD);
        moveToCoords(0, 20, true);
        turn(135);
        moveToCoords(0, 5, true);
        while (!isStopRequested()){}
    }
}
