package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


// This code executes the second test (3 Second Turn Test).
// The full specification is in the Engineering Notebook.
@Autonomous(name = "Test 2: 3 Second Turn Test", group = "Linear Opmode")
public class Turn extends LinearOpMode {

    private Robot robot  = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    //TODO Comment out this code if you don't have Rev Expansion Hub
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //TODO Comment out this code if you don't have Rev Expansion Hub
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuInit();

        waitForStart();
        runtime.reset();// Turn the robot for 3000 ms
        while (runtime.milliseconds() < 3000 && opModeIsActive()) {
            robot.leftFrontMotor.setPower(-1);
            robot.leftBackMotor.setPower(-1);
            robot.rightFrontMotor.setPower(1);
            robot.rightBackMotor.setPower(1);

            //TODO Comment out this code if you don't have Rev Expansion Hub (this code also assumes flat mount of REV Expansion Hub)
            Orientation angles = imu.getAngularOrientation (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Spin Degree : ",String.format(Locale.getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }
        // Stop the robot
        robot.stopRobot();

        //TODO Comment out this code if you don't have Rev Expansion Hub (this code also assumes flat mount of REV Expansion Hub)
        while (runtime.milliseconds() < 6000 && opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Spin Degree : ",String.format(Locale.getDefault(), "%.1f", angles.firstAngle*-1));
            telemetry.update();
        }

    }

    //TODO Comment out this code if you don't have Rev Expansion Hub
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

