package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmBenderSubsystem {

    static DcMotor armMotor;
    public static void initialize(@NonNull HardwareMap HwMap) {
        armMotor = HwMap.get(DcMotor.class,"armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void extendArm(float speed) {
        armMotor.setPower(speed);
    }



}
