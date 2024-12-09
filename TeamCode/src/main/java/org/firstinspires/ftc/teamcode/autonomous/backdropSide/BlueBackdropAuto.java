package org.firstinspires.ftc.teamcode.autonomous.backdropSide;

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
import org.firstinspires.ftc.teamcode.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.autonomous.backdropSide.BackdropSidePositions;
import org.firstinspires.ftc.teamcode.commands.Intake.OuttakeAutoCommand;
import org.firstinspires.ftc.teamcode.commands.Intake.StartIntakeForStack;
import org.firstinspires.ftc.teamcode.commands.Intake.StopIntake;
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

@Autonomous(name = "blue backdrop", group = ".")
public class BlueBackdropAuto extends AutoBase {

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
//                .afterTime(0.6, robot.deposit::deposit)
//                .waitSeconds(1.6)
//                .afterTime(0d, robot.deposit::stop)
//                .afterTime(0d, () -> {retractCommand.schedule();}) // se termina pixelul galben
                .afterTime(0.6, () -> autoScoreCommand.schedule())
                .waitSeconds(1.6)
                .setReversed(false)
                .splineToLinearHeading(BackdropSidePositions.blueIntermediarPos, -Math.PI / 2, lowVelConstraint)
                .afterTime(1d, () -> {startIntakeForStackCommand.schedule();})
                .lineToY(BackdropSidePositions.blueStackPos.position.y + 10, midVelConstraint)
                .lineToY(BackdropSidePositions.blueStackPos.position.y, lowVelConstraint)
                .turn(Math.toRadians(-10))
                .waitSeconds(3)
                .setReversed(true)
                .afterTime(1d, () -> {stopIntakeCommand.schedule();})
                .lineToY(BackdropSidePositions.blueIntermediarPos.position.y - 10, midVelConstraint)
                .afterTime(0, () -> midExtendCommand.schedule())
                .afterTime(1d, robot.arm::goLow)
                .lineToY(BackdropSidePositions.blueIntermediarPos.position.y)
                .splineToLinearHeading(BackdropSidePositions.blueBackdropPos[2], Math.PI / 2, lowVelConstraint)
                .afterTime(2d, () -> autoScoreCommand.schedule())
                .waitSeconds(3d)
                .build();
    }
}
