package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


// This code is used for tests 3 to 5.
// The full specification is in the Engineering Notebook.
@TeleOp(name = "Test 3 to 5: Teleop", group = "Linear Opmode")
public class Teleop extends OpMode {

    private Robot robot  = new Robot();
    private double leftThrottle, rightThrottle = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    // Basic Tank Drive to control the robot.
    @Override
    public void loop(){
        leftThrottle = -gamepad1.right_stick_y;
        rightThrottle = -gamepad1.left_stick_y;

        robot.leftFrontMotor.setPower(leftThrottle);
        robot.leftBackMotor.setPower(leftThrottle);
        robot.rightFrontMotor.setPower(rightThrottle);
        robot.rightBackMotor.setPower(rightThrottle);
        }


    @Override
    public void stop() {
    }
}

