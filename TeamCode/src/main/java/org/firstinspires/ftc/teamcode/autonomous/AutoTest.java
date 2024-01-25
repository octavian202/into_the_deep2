package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.backdropSide.BackdropSidePositions;
import org.firstinspires.ftc.teamcode.commands.Intake.OuttakeAutoCommand;
import org.firstinspires.ftc.teamcode.commands.LowExtendCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;
import org.firstinspires.ftc.teamcode.movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SpikeDetectionCamera;
import org.firstinspires.ftc.teamcode.vision.pipelines.ThresholdPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "auto test", group = ".")
public class AutoTest extends AutoBase {

    @Override
    protected void setColor() {
        color = "blue";
    }

    @Override
    protected void afterInit() {
        Pose2d spikePos = BackdropSidePositions.blueSpikePos[spikePosition - 1];
        Pose2d backdropPos = BackdropSidePositions.blueBackdropPos[spikePosition - 1];

        driveAction = robot.mecanumDrive.actionBuilder(startPose)
                // merge la spike
                .splineToLinearHeading(spikePos, spikePos.heading)
                .afterTime(0, () -> { // pune pixelul
                    OuttakeAutoCommand outtakeAutoCommand = new OuttakeAutoCommand(robot.intake);
                    outtakeAutoCommand.schedule();
                })
                .waitSeconds(1.6) // iese pixelul
                .afterTime(0.2, () -> {lowExtendCommand.schedule();})
                .setReversed(true)
                .splineToLinearHeading(backdropPos, backdropPos.heading, lowVelConstraint)
                .afterTime(0.4, robot.deposit::deposit)
                .waitSeconds(1.4)
                .afterTime(0d, robot.deposit::stop)
                .afterTime(0d, () -> {retractCommand.schedule();})
                .splineToLinearHeading(BackdropSidePositions.blueParkOutsidePos, Math.PI / 2, lowVelConstraint)
                .build();
    }
}
