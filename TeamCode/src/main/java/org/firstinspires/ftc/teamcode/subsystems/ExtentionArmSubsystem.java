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
    }
    public void turnMotor(float speed) {
        extensionMotor.setPower(speed);
    }
    public void handleControllerInput(Gamepad gamepad) {
        float y = gamepad.left_stick_y;
        turnMotor(y);
    }

}
