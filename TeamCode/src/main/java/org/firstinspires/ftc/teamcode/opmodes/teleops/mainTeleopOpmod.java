package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmBenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtentionArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

@TeleOp
public class mainTeleopOpmod extends OpMode {
    double lastTime;
    DriveSubsystem drive_base = new DriveSubsystem();
    PivotSubsystem pivot = new PivotSubsystem();
    ArmBenderSubsystem armBender = new ArmBenderSubsystem();

    EndEffectorSubsystem intake = new EndEffectorSubsystem();

    @Override

    public void init() {
        drive_base.initialize(hardwareMap);
        pivot.initialize(hardwareMap);
        armBender.initialize(hardwareMap);
        intake.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        lastTime = getRuntime();
        drive_base.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true);
        drive_base.updatePosition(getRuntime() - lastTime);

        pivot.turnPivot(gamepad2.left_stick_y);

        armBender.extendArm(gamepad2.right_stick_y);


        while (gamepad2.right_bumper) {
            intake.runIntake(1);
        }
        while (gamepad2.left_bumper){
            intake.runIntake(-1);
        }



    }
}
