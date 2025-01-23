package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtentionArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

public class mainTeleopOpmod extends OpMode {
    @Override
    public void init() {
        DriveSubsystem.initialize(hardwareMap);
        PivotSubsystem.initialize(hardwareMap);
        ExtentionArmSubsystem.initialize(hardwareMap);
        EndEffectorSubsystem.initialize(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
