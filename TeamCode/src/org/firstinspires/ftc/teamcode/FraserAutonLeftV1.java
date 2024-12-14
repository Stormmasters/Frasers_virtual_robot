package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
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
@TeleOp
public class FraserAutonLeftV1 extends OpMode{
    double LX, LY, RX, sensitivity = 0.5, a, b;
    DcMotor BL, FL, FR, BR, S1, S2, M1, M2;
    Servo Intake, Arm, Wrist;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init(){
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
        Intake = hardwareMap.get(Servo.class, "claw");
        Arm = hardwareMap.get(Servo.class, "shoulder");
        Wrist = hardwareMap.get(Servo.class, "wrist");
        runtime.reset();
        BL.setPower(0.5);
        FL.setPower(0.5);
        FR.setPower(0.5);
        BR.setPower(0.5);
        S1.setPower(0.5);
        S2.setPower(-0.5);
        Arm.setPosition(0.1);
        while (runtime.seconds() < 1){}
        BL.setPower(-0.5);
        FL.setPower(-0.5);
        FR.setPower(0.5);
        BR.setPower(0.5);
        while (runtime.seconds() < 1.8){}
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        while (runtime.seconds() < 3.5){}
        Arm.setPosition(0.5);
        while (runtime.seconds() < 5){}
        BL.setPower(0.5);
        FL.setPower(0.5);
        FR.setPower(0.5);
        BR.setPower(0.5);
        while (runtime.seconds() < 5.9){}
        S1.setPower(0);
        S2.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        Intake.setPosition(0.25);
        S1.setPower(0);
        S2.setPower(0);
        while (runtime.seconds() < 1);
        S1.setPower(0);
        S2.setPower(0);
        BL.setPower(-0.2);
        FL.setPower(-0.2);
        BR.setPower(-0.2);
        FR.setPower(-0.2);
        while (runtime.seconds() < 2.5);
        S1.setPower(-0.4);
        S2.setPower(-0.4);
        BL.setPower(-0.3);
        FL.setPower(-0.3);
        BR.setPower(-0.3);
        FR.setPower(-0.3);
        while (runtime.seconds() < 1.5);
        S1.setPower(-0.3);
        S2.setPower(-0.3);
        BL.setPower(-0.3);
        FL.setPower(-0.3);
        BR.setPower(0.3);
        FR.setPower(0.3);
        while (runtime.seconds() < 2.5);
        BL.setPower(0.3);
        FL.setPower(-0.3);
        BR.setPower(-0.3);
        FR.setPower(-0.3);
        S1.setPower(0);
        S2.setPower(0);
        Arm.setPosition(0.3);
        Wrist.setPosition(0.05);
        while (runtime.seconds() < 3.5);
        
    }
    @Override
    public void loop(){
    }
}
