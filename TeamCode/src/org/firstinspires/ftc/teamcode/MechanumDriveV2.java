package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
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
public class MechanumDriveV2 extends OpMode{
    double LX, LY, RX, sensitivity = 0.5;
    DcMotor BL, FL, FR, BR, Arm;
    Servo S1;
    @Override
    public void init(){
        BL = hardwareMap.dcMotor.get("back_left_motor");
        FL = hardwareMap.dcMotor.get("front_left_motor");
        FR = hardwareMap.dcMotor.get("front_right_motor");
        BR = hardwareMap.dcMotor.get("back_right_motor");
        Arm = hardwareMap.dcMotor.get("arm_motor");
        S1 = hardwareMap.servo.get("hand_servo");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // This matches the DcMotors to the ones configured in the Control Hub
        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);
        FR.setDirection(FORWARD);
        BR.setDirection(FORWARD);
        Arm.setDirection(FORWARD);
        S1.setDirection(Servo.Direction.FORWARD);
    }
    @Override
    public void loop(){
        telemetry.addLine("Controls:");
        LX = -gamepad1.left_stick_x * sensitivity;
        LY = -gamepad1.left_stick_y * sensitivity;
        RX = -gamepad1.right_stick_x * sensitivity;
        if (sensitivity < 1 && gamepad1.dpad_up){
            sensitivity += 0.01;
        }
        else if (sensitivity > 0 && gamepad1.dpad_down){
            sensitivity -= 0.01;
        }
        telemetry.addLine("Hold dpad up and dpad down to raise and lower sensitivity; Sensitivity: " + String.format("%.2g%n", sensitivity));
        BL.setPower(LY + LX - RX);
        FL.setPower(LY - LX - RX);
        FR.setPower(LY + LX + RX);
        BR.setPower(LY - LX + RX);
        telemetry.addLine("Hold left trigger and right trigger to raise and lower the arm; Arm position: " + Arm.getCurrentPosition());
        Arm.setPower((gamepad1.left_trigger - gamepad1.right_trigger) * 0.5);
        if (gamepad1.left_bumper){
            S1.setPosition(S1.getPosition() + 0.01);
        }
        else if (gamepad1.right_bumper){
            S1.setPosition(S1.getPosition() - 0.01);
        }
    }
}