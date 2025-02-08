package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotSubsystem {

    static double TICKSPERDEGREE = 28 // number of encoder ticks per rotation of the bare motor
            * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
            * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
            * 1/360.0; // we want ticks per degree, not per rotation

       static  DcMotor pivotMotor;
    public static void initialize(@NonNull HardwareMap HwMap) {
        pivotMotor = HwMap.get(DcMotor.class,"extensionMotor");
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void turnPivot(float speed) {
        pivotMotor.setPower(speed);
    }


    public static void goToPosition (double targetAngle){
        double currentPosition = pivotMotor.getCurrentPosition();
        double currentAngle = currentPosition % TICKSPERDEGREE;
        double targetPosition = targetAngle*TICKSPERDEGREE;

        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setTargetPosition((int) (targetPosition));

        if (targetAngle - currentAngle > 5)
            pivotMotor.setPower(1);
        else {
            pivotMotor.setPower(0);
            pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }
}
