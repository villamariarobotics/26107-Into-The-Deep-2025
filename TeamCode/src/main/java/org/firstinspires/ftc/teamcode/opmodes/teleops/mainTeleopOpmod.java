package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtentionArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

public class mainTeleopOpmod extends OpMode {
    DriveSubsystem drive_base = new DriveSubsystem();
    PivotSubsystem pivot = new PivotSubsystem();
    ExtentionArmSubsystem extension = new ExtentionArmSubsystem();
    EndEffectorSubsystem end_effector = new EndEffectorSubsystem();
    @Override
    public void init() {
        drive_base.initialize(hardwareMap);
        pivot.initialize(hardwareMap);
        extension.initialize(hardwareMap);
        end_effector.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        drive_base.handleControllerInput(gamepad1);
        extension.handleControllerInput(gamepad2);

    }
}
