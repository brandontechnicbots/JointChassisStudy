package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// This code executes test 6 (Straight Line Drift Test).
// The full specification is in the Engineering Notebook.
@Autonomous(name = "Test 6: Straight Line Drift Test", group = "Linear Opmode")
public class DriftTest extends LinearOpMode {

    private Robot robot  = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // For Neverest 40
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private double targetTicks = 120 * COUNTS_PER_INCH; // 120 in converted to counts

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        // Reset the encoder on front left motor
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run the robot forward for the target number of ticks
        while (robot.leftFrontMotor.getCurrentPosition() < targetTicks && opModeIsActive()) {
            robot.leftFrontMotor.setPower(1);
            robot.leftBackMotor.setPower(1);
            robot.rightFrontMotor.setPower(1);
            robot.rightBackMotor.setPower(1);
        }
        // Stop the robot
        robot.stopRobot();
    }
}
