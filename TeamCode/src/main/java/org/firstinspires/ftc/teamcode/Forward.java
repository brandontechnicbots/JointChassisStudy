package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


// This code executes the first test (Driving forward for 5 seconds).
// The full specification is in the Engineering Notebook.
@Autonomous(name = "Test 1: Forward Speed Test", group = "Linear Opmode")
public class Forward extends LinearOpMode {

    private Robot robot  = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        runtime.reset();

        // Run the robot forward for 5000 ms
        while (runtime.milliseconds() < 5000 && opModeIsActive()) {
            robot.leftFrontMotor.setPower(1);
            robot.leftBackMotor.setPower(1);
            robot.rightFrontMotor.setPower(1);
            robot.rightBackMotor.setPower(1);
        }
        // Stop the robot
        robot.stopRobot();
    }
}

