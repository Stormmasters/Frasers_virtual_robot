package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class Homework1 extends OpMode {
    double angle;

    public Homework1(double angle) {
        this.angle = angle;
    }
    double X;
    double getX(){
        return this.X;
    }
    void setX(double NewX){
        this.X = NewX;
    }
    double changeX(double C){
        return this.X += C;
    }
    double Y;
    double getY(){
        return this.Y;
    }

    void setY(double NewY){
        this.Y = NewY;
    }

    double changeY(double C){
        return this.Y += C;
    }

    public double getAngle(){
        return this.angle;
    }

    public double getHeading() {
        double angle = this.angle;
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }
    @Override
    public String toString() {
        return "RobotLocation: angle (" + angle + ")";
    }

    public void turn(double angleChange) {
        angle += angleChange;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }
    @Override
    public void init(){
    }
    @Override
    public void loop(){
        telemetry.addData("Angle: ", angle);
        if (gamepad1.dpad_left == true){
            changeX(-0.1);
        }
        if (gamepad1.dpad_right == true){
            changeX(0.1);
        }
        if (gamepad1.dpad_down == true){
            changeY(-0.1);
        }
        if (gamepad1.dpad_up == true){
            changeY(0.1);
        }
    }
}