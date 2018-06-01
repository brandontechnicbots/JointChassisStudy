package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


// This code executes the first test (Driving forward for 6 seconds).
// The full specification is in the Engineering Notebook.
@Autonomous(name = "Test 1: Forward Speed Test", group = "Linear Opmode")
public class Forward extends LinearOpMode {

    private DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        waitForStart();
        runtime.reset();

        // Run the robot forward for 6000 ms
        while (runtime.milliseconds() < 6000 && opModeIsActive()) {
            leftFrontMotor.setPower(1);
            leftBackMotor.setPower(1);
            rightFrontMotor.setPower(1);
            rightBackMotor.setPower(1);
        }
        // Stop the robot
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}

