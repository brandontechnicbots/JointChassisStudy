package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


// This code executes the second test (3 Second Turn Test).
// The full specification is in the Engineering Notebook.
@Autonomous(name = "Test 2: 3 Second Turn Test", group = "Linear Opmode")
public class Turn extends LinearOpMode {

    private Robot robot  = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        runtime.reset();

        // Turn the robot for 3000 ms
        while (runtime.milliseconds() < 3000 && opModeIsActive()) {
            robot.leftFrontMotor.setPower(-1);
            robot.leftBackMotor.setPower(-1);
            robot.rightFrontMotor.setPower(1);
            robot.rightBackMotor.setPower(1);
        }
        // Stop the robot
        robot.stopRobot();
    }
}

