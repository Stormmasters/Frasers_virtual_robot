package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp

public class FieldOrientedMechanum extends OpMode {
    DcMotor backLeft, frontLeft, frontRight, backRight;

    public void forward1() {
    }

    @Override
    public void init() {
        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        backRight = hardwareMap.dcMotor.get("back_right_motor");
        backLeft.setDirection(REVERSE);
        frontLeft.setDirection(REVERSE);
        frontRight.setDirection(FORWARD);
        backRight.setDirection(FORWARD);
        double x;
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
        forward1();
    }

    double BackLeft;
    double FrontLeft;
    double FrontRight;
    double BackRight;
    double RightY;
    double RightX;
    double LeftX;
    double LeftY;
    double DFromOrigin;
    double RAngle;
    double LAngle;
    double HeadingR;
    double LAngleA;
    double LeftXA;
    double LeftYA;
    double m = 0.5;
    double r;
    boolean fieldOriented = true;

    @Override
    public void loop() {
        RightY = m * gamepad1.right_stick_y;
        RightX = m * gamepad1.right_stick_x;
        LeftX = m * gamepad1.left_stick_x;
        LeftY = m * gamepad1.left_stick_y;
        DFromOrigin = Math.sqrt(LeftY * LeftY + LeftX * LeftX);
        telemetry.addData("Distance from origin: ", DFromOrigin);
        RAngle = Math.atan2(RightY, RightX) + Math.PI * 2;
        LAngle = Math.atan2(LeftY, LeftX);
        telemetry.addData("Angle (radians): ", RAngle);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        HeadingR = Math.toRadians(orientation.firstAngle * 180 / Math.PI) + Math.PI;
        telemetry.addData("Heading in radians is: ", HeadingR);
        LAngleA = LAngleA - HeadingR;
        r = Math.hypot(LeftY, LeftX);
        if (LAngleA > 2 * Math.PI) {
            LAngleA -= 2 * Math.PI;
        }
        telemetry.addData("Input Angle Difference: ", LAngleA);
        LeftXA = r * Math.cos(LAngleA);
        LeftYA = r * Math.sin(LAngleA);
        telemetry.addData("Adjusted X: ", LeftXA);
        telemetry.addData("Adjusted Y: ", LeftYA);
        if (RightX != 0) {
            backLeft.setPower(-RightX);
            frontLeft.setPower(-RightX);
            frontRight.setPower(RightX);
            backRight.setPower(RightX);
        }
        if (fieldOriented) {
            if (LeftY != 0 || LeftX != 0) {
                BackLeft = LeftYA + LeftXA;
                FrontLeft = LeftYA - LeftXA;
                FrontRight = LeftYA + LeftXA;
                BackRight = LeftYA - LeftXA;
                backLeft.setPower(BackLeft);
                frontLeft.setPower(FrontLeft);
                frontRight.setPower(FrontRight);
                backRight.setPower(BackRight);
            } else {
                backLeft.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);

            }
        } else {
            if (LeftY != 0 || LeftX != 0) {
                BackLeft = LeftY + LeftX;
                FrontLeft = LeftY - LeftX;
                FrontRight = LeftY + LeftX;
                BackRight = LeftY - LeftX;
                backLeft.setPower(BackLeft);
                frontLeft.setPower(FrontLeft);
                frontRight.setPower(FrontRight);
                backRight.setPower(BackRight);
            } else {
                backLeft.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }
        }
    }
}