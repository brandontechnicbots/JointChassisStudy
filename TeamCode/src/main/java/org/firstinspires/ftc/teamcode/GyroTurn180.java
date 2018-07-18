package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

// This code executes the 7th test (90 degree turn offset).
// The full specification is in the Engineering Notebook.
@Autonomous(name = "Test 7: 160 degree turn offset", group = "Linear Opmode")
public class GyroTurn180 extends LinearOpMode {

    private Robot robot  = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private static double TURN_P = 0.01;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuInit();

        waitForStart();
        runtime.reset();

        // Turn the robot for 160 degrees
        gyroTurn(160);

        // Stop the robot
        robot.stopRobot();
    }

    private void gyroTurn(double deg) {
        double target_angle = getHeading() - deg;
        while (Math.abs((target_angle - getHeading())% 360) > 3 && opModeIsActive()) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute Error
            double motor_output = Range.clip(error_degrees * TURN_P, -.6 ,.6); //Get Correction
            // Send corresponding powers to the motors
            robot.leftFrontMotor.setPower(1 * motor_output);
            robot.leftBackMotor.setPower(1 * motor_output);
            robot.rightFrontMotor.setPower(-1*motor_output);
            robot.rightBackMotor.setPower(-1*motor_output);

            Orientation angles = imu.getAngularOrientation (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Spin Target : ",target_angle);
            telemetry.addData("Spin Degree : ",String.format(Locale.getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }

        robot.stopRobot();

        //extra 6 seconds to display the values for reading
        while (runtime.milliseconds() < 6000 && opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Spin Target : ",target_angle);
            telemetry.addData("Spin Degree : ",String.format(Locale.getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }
    }

    private float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    /* Initializes Rev Robotics IMU */
    private void imuInit() {
        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "imu";
        imu.initialize(parameters);
    }
}
