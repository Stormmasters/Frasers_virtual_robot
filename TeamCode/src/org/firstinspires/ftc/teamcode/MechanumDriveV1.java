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
public class MechanumDriveV1 extends OpMode{
    DcMotor BL, FL, FR, BR, Arm;
    Servo S1;
    double LX, LY, RX, LXa, LYa, Angle, AngleA, r, sensitivity = 0.5, armPos, maxArmPos = 2240, maxServoPos = 4, startAngle = 0;
    boolean fieldOriented = true, invControls = false, moveToStart = false, moveToMax = false;
    @Override
    public void init(){
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        // Configures IMU
        imu.resetYaw();
        // "Calibrates" IMU by making the current heading 0
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
        // Sets motor directions
    }
    @Override
    public void loop(){
        telemetry.addLine("Drive Controls: ");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        // Gets the robot's current orientation, reconfigures
        if (gamepad1.left_bumper){
            if (fieldOriented){
                fieldOriented = false;
            }
            else{
                fieldOriented = true;
            }
        }
        // This code allows the left bumper to switch whether fieldOriented is true
        if (gamepad1.right_bumper){
            if (invControls){
                invControls = false;
            }
            else{
                invControls = true;
            }
        }
        // This code allows the right bumper to switch whether invControls is true
        if (sensitivity < 1 && gamepad1.dpad_up){ // If sensitivity isn't at its max and the button is pressed:
            sensitivity += 0.0004; // Increase sensitivity
        }
        else if (sensitivity > 0 && gamepad1.dpad_down){ // If sensitivity isn't at its min and the button is pressed:
            sensitivity -= 0.0004; // Decrease sensitivity
        }
        if (sensitivity > 1){ // In case it doesn't update in time
            sensitivity = 1;
        }
        else if (sensitivity < 0){
            sensitivity = 0;
        }
        telemetry.addLine("Press dpad down to lower the sensitivity and dpad up to raise it");
        // Right trigger makes sensitivity go up and vice versa
        if (gamepad1.b){ // If b is pressed
            startAngle = -orientation.firstAngle + startAngle + 2 * Math.PI; // The startAngle, or the difference between the current orientation and the starting orientation
            // Is equal to the negative yaw plus itself plus 2pi (2pi because this is in radians)
            while (startAngle > 2 * Math.PI){ // If it's greater than 2pi, keep subtracting 2pi til it's within 2pi
                startAngle = startAngle - 2 * Math.PI;
            }
            imu.resetYaw(); // Resets yaw
        }
        telemetry.addLine("Press b to reset the IMU");
        telemetry.addLine("The robot is oriented " + Math.toDegrees(startAngle) + " degrees from the starting orientation");
        LX = sensitivity * -gamepad1.left_stick_x;
        LY = sensitivity * -gamepad1.left_stick_y;
        RX = sensitivity * -gamepad1.right_stick_x;
        // I like to turn controller inputs into smaller variables because "gamepad1.ect." is such a mouthful
        // It also multiplies them by "sensitivity", which allows the driver to change the sensitivity
        if (invControls){
            LX *= -1;
            LY *= -1;
            RX *= -1;
            telemetry.addLine("Drving is inverted; Press the right bumper to disable inverted controls");
        }
        else{
            telemetry.addLine("Drving isn't inverted; Press the right bumper to enable inverted controls");
        }
        // If controls are inverted, make the inputs negative
        if (fieldOriented){
            r = Math.hypot(LY, LX);
            // Getting the "r", or distance of the controller input from the origin
            Angle = Math.atan2(LY, LX);
            // "atan2" converts a point into an angle from the origin
            AngleA = -orientation.firstAngle - Angle;
            // Difference of -yaw and Angle with the 0.5 * pi added because it starts out offset by 90 degrees
            LYa = -Math.sin(AngleA) * r;
            LXa = Math.cos(AngleA) * r;
            // The adjusted inputs are multiplied by r in order to get the magnitude of a movement, allowing for speed control
            telemetry.addLine("Driving is field oriented; Press the left bumper to disable field oriented control");
        }
        else{
            LYa = LY;
            LXa = LX;
            // If fieldOriented is turned off, the adjusted inputs are just set to the regular ones
            telemetry.addLine("Driving is robot oriented; Press the left bumper to enable field oriented control");
        }
        BL.setPower(LYa + LXa - RX);
        FL.setPower(LYa - LXa - RX);
        FR.setPower(LYa + LXa + RX);
        BR.setPower(LYa - LXa + RX);
        // Sets motor powers
        telemetry.addLine("Sensitivity: " + String.format("%.2g%n", sensitivity));
        if (gamepad1.left_trigger > 0 && Arm.getCurrentPosition() < maxArmPos){ // If left trigger is pressed and arm position < max
            Arm.setPower(0.5 * gamepad1.left_trigger); // Set the motor power to the trigger input
            moveToStart = false;
            moveToMax = false;
            // Stop moving to a position
        }
        else if (gamepad1.right_trigger > 0 && Arm.getCurrentPosition() > 0){ // If right trigger and position > min
            Arm.setPower(-0.5 * gamepad1.right_trigger); // Set the motor power to the trigger input
            moveToStart = false;
            moveToMax = false;
            S1.setPosition(0); // Setting servo position back to default if the arm comes back down
        }
        else{
            Arm.setPower(0); // If no other inputs, set power to 0
        }
        if (gamepad1.dpad_left && !moveToStart){ // If left dpad, enable moveToStart
            moveToStart = true;
            moveToMax = false;
        }
        if (gamepad1.dpad_right && !moveToMax){ // If left dpad, enable moveToMax
            moveToMax = true;
            moveToStart = false;
        }
        if (gamepad1.a){ // If a is pressed, stop moving to either position
            moveToStart = false;
            moveToMax = false;
        }
        telemetry.addLine("Auxiliary controls: ");
        telemetry.addLine("Use left and right triggers to raise and lower arm");
        telemetry.addLine("Use left and right dpads to automatically move to max or min");
        if (Arm.getCurrentPosition() >= maxArmPos){ // If it's at max position, display max position reahced
            telemetry.addLine("Arm position: " + maxArmPos + "; Max arm position reached");
        }
        else if (Arm.getCurrentPosition() <= 0){ // If it's at min position, display min position reahced
            telemetry.addLine("Arm position: 0; Min arm position reached");
        }
        else{ // If not at max or min, just display arm position
            telemetry.addLine("Arm position: " + Arm.getCurrentPosition());
        }
        if (moveToStart && Arm.getCurrentPosition() > 0){ // If moving to start, set power as such and set servo to default position
            telemetry.addLine("Moving to min position");
            telemetry.addLine("Press 'a' to terminate movement");
            // Displays "moving to position" and how to stop that
            Arm.setPower(-0.5);
            S1.setPosition(0);
        }
        else{ // If arm position is no longer greater than zero, stop moving to start
            moveToStart = false;
        }
        if (moveToMax && Arm.getCurrentPosition() < maxArmPos){ // Same thing as moveToStart
            telemetry.addLine("Moving to max position");
            telemetry.addLine("Press 'x' to terminate movement");
            Arm.setPower(0.5);
        }
        else{
            moveToMax = false;
        }
        if (Arm.getCurrentPosition() >= maxArmPos || Arm.getCurrentPosition() <= 0){ // If arm is at max or min
            telemetry.addLine("Hold 'y' to close claw and 'x' to open claw"); // Explains controls
            if (S1.getPosition() > maxServoPos){ // If position > max set to max
                S1.setPosition(maxServoPos);
            }
            else if (S1.getPosition() < 0){ // Vice versa
                S1.setPosition(0);
            }
            if (gamepad1.y && S1.getPosition() < maxServoPos){ // If "y" and pos < mas, increase pos
                S1.setPosition(S1.getPosition() + 0.01);
            }
            else if (gamepad1.x && S1.getPosition() > 0){ // Vice versa
                S1.setPosition(S1.getPosition() - 0.01);
            }
            if (S1.getPosition() >= maxServoPos){ // Tells user servo at max
                telemetry.addLine("Current servo position: " + maxServoPos + "; Max servo position reached");
            }
            else if (S1.getPosition() <= 0){ // Tells user servo at min
                telemetry.addLine("Current servo position: 0.0; Min servo position reached");
            }
            else{ // Tells user servo position
                telemetry.addLine("Current servo position: " + String.format("%.2g%n", S1.getPosition()));
            }
        }
        else{ // If not at max or min, says move to max or min to use servo
            telemetry.addLine("Arm must be at max or min to enable the servo");
        }
    }
}