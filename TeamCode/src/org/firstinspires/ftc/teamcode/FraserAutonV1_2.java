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
public class FraserAutonV1_2 extends LinearOpMode {
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
    public void moveUntil(double distance, boolean forward, String sensor){
        FD.getDistance(DistanceUnit.INCH);
        BD.getDistance(DistanceUnit.INCH);
        if (FD.getDistance(DistanceUnit.INCH) < 60){
            yDistance = "FD";
        }
        else if (BD.getDistance(DistanceUnit.INCH) < 60){
            yDistance = "BD";
        }
        while (!isStopRequested()){
            if (forward){
                allM_Power(0.3);
            }
            else {
                allM_Power(-0.3);
            }
            if (sensor.equals("FD") && FD.getDistance(DistanceUnit.INCH) < distance + 3){
                if (forward){
                    allM_Power(0.1);
                }
                else {
                    allM_Power(-0.1);
                }
                if (FD.getDistance(DistanceUnit.INCH) < distance + 0.1){
                    if (forward){
                        allM_Power(0.01);
                    }
                    else {
                        allM_Power(-0.01);
                    }
                    if (FD.getDistance(DistanceUnit.INCH) < distance + 0.01){
                        System.out.println("break");
                        break;
                    }
                }
            }
            else if (sensor.equals("BD") && BD.getDistance(DistanceUnit.INCH) > distance - 3){
                if (forward){
                    allM_Power(0.1);
                }
                else {
                    allM_Power(-0.1);
                }
                if (BD.getDistance(DistanceUnit.INCH) > distance - 0.1){
                    if (forward){
                        allM_Power(0.01);
                    }
                    else {
                        allM_Power(-0.01);
                    }
                    if (BD.getDistance(DistanceUnit.INCH) > distance - 0.01){
                        System.out.println("break1");
                        break;
                    }
                }
            }
        }
        allM_Power(0);
        System.out.println("hi");
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
        moveUntil(20, true, "BD");
        turn(90);
        moveUntil(20, true, "FD");
        turn(45);
        while (!isStopRequested()){}
    }
}
