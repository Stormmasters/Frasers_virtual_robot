package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class FraserAutonLeftV1 extends LinearOpMode{
    double LX, LY, RX, sensitivity = 0.5, a, b;
    DcMotor BL, FL, FR, BR, S1, S2, M1, M2;
    Servo Intake, Arm, Wrist;
    private ElapsedTime runtime = new ElapsedTime();
    public void MoveUntil(double speed, double time, String direction){
        runtime.reset();
        if (direction.equals("left")){
            BL.setPower(speed);
            FL.setPower(-speed);
            FR.setPower(speed);
            BR.setPower(-speed);
        }
        else if (direction.equals("right")){
            BL.setPower(-speed);
            FL.setPower(speed);
            FR.setPower(-speed);
            BR.setPower(speed);
        }
        else if (direction.equals("forward")){
            BL.setPower(speed);
            FL.setPower(speed);
            FR.setPower(speed);
            BR.setPower(speed);
        }
        else if (direction.equals("backward")){
            BL.setPower(-speed);
            FL.setPower(-speed);
            FR.setPower(-speed);
            BR.setPower(-speed);
        }
        while (runtime.seconds() < time && !isStopRequested()){}
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
    public void Turn(double speed, double time){
        runtime.reset();
        BL.setPower(speed);
        FL.setPower(speed);
        FR.setPower(-speed);
        BR.setPower(-speed);
        while (runtime.seconds() < time && !isStopRequested()){}
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
    @Override
    public void runOpMode(){
        BL = hardwareMap.dcMotor.get("back_left_motor");
        FL = hardwareMap.dcMotor.get("front_left_motor");
        FR = hardwareMap.dcMotor.get("front_right_motor");
        BR = hardwareMap.dcMotor.get("back_right_motor");
        S1 = hardwareMap.dcMotor.get("left_slide");
        S2 = hardwareMap.dcMotor.get("right_slide");
        S1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        S2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        S1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // This matches the DcMotors to the ones configured in the Control Hub
        FL.setDirection(REVERSE);
        S2.setDirection(REVERSE);
        Intake = hardwareMap.get(Servo.class, "claw");
        Arm = hardwareMap.get(Servo.class, "shoulder");
        Wrist = hardwareMap.get(Servo.class, "wrist");
        runtime.reset();
        MoveUntil(0.5, 0.5, "right");
        Turn(-0.5, 0.35);
        MoveUntil(0.5, 1.1, "right");
        MoveUntil(0.5, 1.2, "forward");
        runtime.reset();
        MoveUntil(0.5, 1.2, "backward");
        Turn(0.5, 1.35);
        MoveUntil(0.5, 0.1, "left");
        MoveUntil(0.5, 1, "forward");
        MoveUntil(0.5, 0.3, "left");
        Turn(0.5, 0.12);
        MoveUntil(0.5, 1.8, "backward");
        MoveUntil(0.5, 1.8, "forward");
        MoveUntil(0.5, 0.8, "left");
        Turn (-0.5, 0.1);
        MoveUntil(0.5, 1.8, "backward");
        MoveUntil(0.5, 1.8, "forward");
        Turn(0.5, 0.8);
        MoveUntil(0.5, 1.2, "forward");
    }
}