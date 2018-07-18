package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class defines the hardware for the robot and its initialization
 */

public class Robot {

    DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor = null;

    /* local OpMode members. */
    private HardwareMap hwMap = null;

    /* Constructor */
    Robot() {

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //If you need to change the configuration, do it here
        leftFrontMotor = hwMap.get(DcMotor.class, "left_front");
        leftBackMotor = hwMap.get(DcMotor.class, "left_back");
        rightFrontMotor = hwMap.get(DcMotor.class, "right_front");
        rightBackMotor = hwMap.get(DcMotor.class, "right_back");

        // Set all motors direction
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors direction
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        stopRobot();

        //Set Brake Behavior
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Utility function to stop the robot
    void stopRobot() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

}
