package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class Mechanum extends OpMode {
    DcMotor backLeft, frontLeft, frontRight, backRight;
    @Override
    public void init() {
        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        backRight = hardwareMap.dcMotor.get("back_right_motor");
    }

    @Override
    public void loop() {
        double RightY = -gamepad1.right_stick_y;
        double RightX = -gamepad1.right_stick_x;
        double LeftX = -gamepad1.left_stick_x;
        double LeftY = -gamepad1.left_stick_y;
        telemetry.addData("Right X: ", RightX);
        telemetry.addData("Right Y: ", RightY);
        double SquareRX = RightX * RightX;
        double SquareRY = RightY * RightY;
        double DFromOrigin = Math.sqrt(SquareRX + SquareRY);
        telemetry.addData("Distance from origin: ", DFromOrigin);
        double RAngle = Math.atan2(RightY, RightX);
        telemetry.addData("Angle (radians): ", RAngle);
        if (RightX != 0 || LeftY != 0 || LeftX != 0){
            if (RightX != 0){
                backLeft.setDirection(FORWARD);
                frontLeft.setDirection(FORWARD);
                frontRight.setDirection(FORWARD);
                backRight.setDirection(FORWARD);
                backLeft.setPower(RightX);
                frontLeft.setPower(RightX);
                frontRight.setPower(RightX);
                backRight.setPower(RightX);
            }
            else{
                backLeft.setDirection(REVERSE);
                frontLeft.setDirection(REVERSE);
                frontRight.setDirection(FORWARD);
                backRight.setDirection(FORWARD);
                double BackLeft = LeftY + LeftX;
                double FrontLeft = LeftY - LeftX;
                double FrontRight = LeftY + LeftX;
                double BackRight = LeftY - LeftX;
                backLeft.setPower(BackLeft);
                frontLeft.setPower(FrontLeft);
                frontRight.setPower(FrontRight);
                backRight.setPower(BackRight);
            }
        }
        else{
            backLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }
}