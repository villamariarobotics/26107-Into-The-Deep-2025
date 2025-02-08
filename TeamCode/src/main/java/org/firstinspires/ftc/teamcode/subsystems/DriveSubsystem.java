package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {

    private IMU gyro;

    private DcMotor front_left_motor;
    private DcMotor front_right_motor;
    private DcMotor back_left_motor;
    private DcMotor back_right_motor;

    /**
     * X position of the robot in encoder ticks.
     * @see #getXPosition(boolean)
     */
    double xPosition;


    double yPosition;

    /**
     * Heading of the robot in the unit of your choice.
     * @see #getHeading(AngleUnit)
     */
    double robotHeading;

    /**
     * Initialize the mecanum drive subsystem, using the HardwareMap object granted by the OpMode.
     * @param hardwareMap HardwareMap Object, from the OpMode
     * @see HardwareMap
     * @see IMU
     * @see DcMotor
     */
    public  void initialize(@NonNull HardwareMap hardwareMap) {

        gyro = hardwareMap.get(IMU.class, "imu");
        gyro.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );

        front_left_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "right_back_motor");

        front_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drive the robot using mecanum wheels.
     * @param x The x value of the joystick.
     * @param y The y value of the joystick.
     * @param z The z value of the joystick.
     * @param fieldOriented Whether or not the robot is field oriented.
     */
    public  void drive(double x, double y, double z, boolean fieldOriented) {

        // Square the joystick values to make the robot more controllable.
        x = x * Math.abs(x);
        y = y * Math.abs(y);
        z = z * Math.abs(z);

        if (fieldOriented) {
            // Get the robot's heading.
            robotHeading = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the joystick values to the robot's heading.
            double xrobot = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
            double yrobot = x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

            // Calculate the power for each motor.
            double frontLeft = - xrobot + yrobot - z;
            double frontRight = xrobot + yrobot + z;
            double backLeft = xrobot + yrobot - z;
            double backRight = - xrobot + yrobot + z;

            // Normalize the power values.
            double max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight)));
            if (max > 1) {
                frontLeft /= max;
                frontRight /= max;
                backLeft /= max;
                backRight /= max;
            }

            // Set the power of each motor.
            front_left_motor.setPower(frontLeft);
            front_right_motor.setPower(frontRight);
            back_left_motor.setPower(backLeft);
            back_right_motor.setPower(backRight);

        } else {
            // Calculate the power for each motor.
            double frontLeft = - x + y - z;
            double frontRight = x + y + z;
            double backLeft = x + y - z;
            double backRight = - x + y + z;

            // Normalize the power values.
            double max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight)));
            if (max > 1) {
                frontLeft /= max;
                frontRight /= max;
                backLeft /= max;
                backRight /= max;
            }

            // Set the power of each motor.
            front_left_motor.setPower(frontLeft);
            front_right_motor.setPower(frontRight);
            back_left_motor.setPower(backLeft);
            back_right_motor.setPower(backRight);
        }
    }

    /**
     * Move the robot to a specific coordinate, operating in field oriented.
     * @param x The x coordinate to move to.
     * @param y The y coordinate to move to.
     */
    public void moveToCoordinate(double x, double y) {
        double deltaX = x - xPosition;
        double deltaY = y - yPosition;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double angle = Math.atan2(deltaY, deltaX);
        double robotAngle = angle - Math.toRadians(getHeading(AngleUnit.DEGREES));
        double xPower = Math.cos(robotAngle);
        double yPower = Math.sin(robotAngle);

        while (distance > 0.1) { // Adjust the threshold as needed
            drive(xPower, yPower, 0, true);
            deltaX = x - xPosition;
            deltaY = y - yPosition;
            distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            robotAngle = angle - Math.toRadians(getHeading(AngleUnit.DEGREES));
            xPower = Math.cos(robotAngle);
            yPower = Math.sin(robotAngle);
        }
        stop();
    }

    /**
     * Update the robot's position.
     * @param deltaTime The time since the last update.
     */
    public void updatePosition(double deltaTime) {
        double heading = getHeading(AngleUnit.RADIANS);
        double distance = (front_left_motor.getCurrentPosition() + front_right_motor.getCurrentPosition() + back_left_motor.getCurrentPosition() + back_right_motor.getCurrentPosition()) / 4.0;
        double deltaX = distance * Math.cos(heading) * deltaTime;
        double deltaY = distance * Math.sin(heading) * deltaTime;
        xPosition += deltaX;
        yPosition += deltaY;
    }

    /**
     * Get the x position of the robot.
     * @param inMeters Whether or not to return the position in meters, and if not, in encoder ticks.
     * @return The x position of the robot.
     */
    public double getXPosition(boolean inMeters) {
        if (inMeters) {
            return xPosition;
        } else {
            return xPosition;
        }
    }

    /**
     * Get the y position of the robot.
     * @param inMeters Whether or not to return the position in meters, and if not, in encoder ticks.
     * @return The y position of the robot.
     */
    public double getYPosition(boolean inMeters) {
        if (inMeters) {
            return yPosition;
        } else {
            return yPosition;
        }
    }

    /**
     * Get the heading of the robot.
     * @param unit The unit to return the heading in.
     * @return The heading of the robot.
     */
    public double getHeading(AngleUnit unit) {
        return gyro.getRobotYawPitchRollAngles().getYaw(unit);
    }

    /**
     * Reset the robot's position.
     */
    public void resetPosition() {
        xPosition = 0;
        yPosition = 0;
    }

    /**
     * Reset the robot's orientation.
     */
    public void resetOrientation() {
        gyro.resetYaw();
    }

    public void stop() {
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
    }
}