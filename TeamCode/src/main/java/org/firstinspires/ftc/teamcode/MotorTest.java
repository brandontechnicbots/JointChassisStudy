package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;


// This code executes the first test (Driving forward for 5 seconds).
// The full specification is in the Engineering Notebook.
@Autonomous(name = "MotorTest", group = "Linear Opmode")
public class MotorTest extends LinearOpMode {

    private TestRobot robot  = new TestRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        runtime.reset();

        // Run the robot forward for 5000 ms
        while (runtime.milliseconds() < 5000 && opModeIsActive()) {
            robot.motor1.setPower(1);
            robot.motor2.setPower(1);

            telemetry.addData("Motor 1 Encoder : ",robot.motor1.getCurrentPosition());
            telemetry.addData("Motor 2 Encoder : ",robot.motor2.getCurrentPosition());
            telemetry.update();

        }
        // Stop the robot
        robot.stopRobot();
        while (runtime.milliseconds() < 30000 && opModeIsActive()) {
            telemetry.addData("Motor 1 Encoder : ", robot.motor1.getCurrentPosition());
            telemetry.addData("Motor 2 Encoder : ", robot.motor2.getCurrentPosition());
            telemetry.update();
        }
    }
}

