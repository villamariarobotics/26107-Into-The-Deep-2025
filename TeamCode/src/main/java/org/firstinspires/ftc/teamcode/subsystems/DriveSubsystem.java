package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem {

    /** About:
     *
     *
     *
     * created by @CharliePrograms, 2025
     * run initialize on initialize,
     * run handleControllerInput on loop,
     * run translateToPoint to translate to a specific point relative to the
     * robot start position.
     * The constants DISTANCE_PER_SECOND and ANGLES_PER_SECOND will need to be measured
     * and updated.
     */

    static DcMotor left_front_motor;
    static DcMotor right_front_motor;
    static DcMotor left_back_motor;
    static DcMotor right_back_motor;
    static Boolean reverseDirections = false;
    // if the robot goes backwards when told to go forwards,
    //set reverseDirections to true.
    public float xPosition = 0; //distance in cm
    public float yPosition = 0; //distance in cm

    public float rotation = 0; //in radians; 0 = starting rotations
    static float DISTANCE_PER_SECOND = 1; // distance (in cm) moved in one second at full power.
    // TO BE MEASURED!!!! ^^^^^^^^^^^^^^^^
    static float ANGLES_PER_SECOND = 1; // angle (in radians) that the robot turns in one second
    // TO BE MEASURED!!!! ^^^^^^^^^^^^^^^^
    public void initialize(HardwareMap hwMap) {
        left_front_motor = hwMap.get(DcMotor.class, "left_front_motor");
        right_front_motor = hwMap.get(DcMotor.class, "right_front_motor");
        left_back_motor = hwMap.get(DcMotor.class, "left_back_motor");
        right_back_motor = hwMap.get(DcMotor.class, "right_back_motor");
        if (!reverseDirections) {
            right_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            right_back_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            left_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            left_back_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void turnMotor(int motor, float speed) {
        /*
        front
        0---1
        |top|    <- motor mapping
        2---3

        */
        switch (motor) {
            case 0:
                left_front_motor.setPower(speed);
            case 1:
                right_front_motor.setPower(speed);
            case 2:
                left_back_motor.setPower(speed);
            case 3:
                right_back_motor.setPower(speed);
        }
    }

    public void handleControllerInput(Gamepad gamepad) {
        /*
        left stick x: rotate
        right stick: move
         */
        double y = -gamepad.right_stick_y;
        double x = gamepad.right_stick_x * 1.1;
        double rx = gamepad.left_stick_x;
        double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;
        left_front_motor.setPower(leftFrontPower);
        left_back_motor.setPower(leftBackPower);
        right_front_motor.setPower(rightFrontPower);
        right_back_motor.setPower(rightBackPower);
        rotation += (float) ((ANGLES_PER_SECOND/50)*rx);
        xPosition += (float) (Math.sin(rotation)*DISTANCE_PER_SECOND*x/(50*distance));
        yPosition += (float) (Math.cos(rotation)*DISTANCE_PER_SECOND*y/(50*distance));
    }

    public void translateToPoint(float xTarget, float yTarget) {
        // moves the robot to position (xTarget,yTarget) (in cm) relative to
        // its starting position, without rotating the robot.
        float deltaX = xTarget-xPosition; // in cm
        float deltaY = yTarget-yPosition; // in cm
        if ((Math.pow(deltaX,2)+Math.pow(deltaY,2))<25) {
            double denominator = Math.max(Math.abs(deltaX) + Math.abs(deltaY), 1);
            double distance = Math.sqrt(Math.pow(deltaY, 2) + Math.pow(deltaX, 2));
            double leftFrontPower = (deltaX + deltaY) / denominator;
            double leftBackPower = (deltaY - deltaX) / denominator;
            double rightFrontPower = (deltaY - deltaX) / denominator;
            double rightBackPower = (deltaX + deltaY) / denominator;
            left_front_motor.setPower(leftFrontPower);
            left_back_motor.setPower(leftBackPower);
            right_front_motor.setPower(rightFrontPower);
            right_back_motor.setPower(rightBackPower);
            xPosition += (float) (Math.sin(rotation)*DISTANCE_PER_SECOND*deltaX/(50*distance));
            yPosition += (float) (Math.cos(rotation)*DISTANCE_PER_SECOND*deltaY/(50*distance));
        } else {
            left_front_motor.setPower(0);
            left_back_motor.setPower(0);
            right_front_motor.setPower(0);
            right_back_motor.setPower(0);
        }

    }

}