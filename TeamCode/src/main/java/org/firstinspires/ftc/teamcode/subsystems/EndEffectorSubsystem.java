package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EndEffectorSubsystem {

    static CRServo IntakeServo;

    public static void initialize(@NonNull HardwareMap HwMap) {
        IntakeServo = HwMap.get(CRServo.class,"intake-servo");
    }


    public static void runIntake(double speed ){
        IntakeServo.setPower(speed);
    }
}
