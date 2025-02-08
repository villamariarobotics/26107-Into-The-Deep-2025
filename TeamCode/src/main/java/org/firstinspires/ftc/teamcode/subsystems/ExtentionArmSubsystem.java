package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtentionArmSubsystem {

    static DcMotor extensionMotor;
    public static void initialize(@NonNull HardwareMap HwMap) {
       extensionMotor = HwMap.get(DcMotor.class,"extensionMotor");
       extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void extendArm(float speed) {
        extensionMotor.setPower(speed);
    }


}
