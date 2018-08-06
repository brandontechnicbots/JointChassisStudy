package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class defines the hardware for the robot and its initialization
 */

public class TestRobot {

    DcMotor motor1 , motor2 = null;

    /* local OpMode members. */
    private HardwareMap hwMap = null;

    /* Constructor */
    TestRobot() {

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //If you need to change the configuration, do it here
        motor1 = hwMap.get(DcMotor.class, "motor1");
        motor2 = hwMap.get(DcMotor.class, "motor2");

        // Set all motors direction
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Set all motors direction
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        stopRobot();

        //Set Brake Behavior
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Utility function to stop the robot
    void stopRobot() {
        motor1.setPower(0);
        motor2.setPower(0);

    }

}
