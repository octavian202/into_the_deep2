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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.stackSide.StackSidePositions;
import org.firstinspires.ftc.teamcode.commands.Intake.OuttakeAutoCommand;
import org.firstinspires.ftc.teamcode.commands.LowExtendCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.lift.AutoControlCommand;
import org.firstinspires.ftc.teamcode.movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.pipelines.ThresholdPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "auto test", group = ".")
public class AutoTest extends AutoBase {
    @Override
    public void onStarted() {
        Pose2d spikePos = StackSidePositions.blueSpikePos[spikePosition - 1];
        Pose2d backdropPos = StackSidePositions.blueBackdropPos[spikePosition - 1];

        Action driveAction = robot.mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
                // merge la spike
                .splineToLinearHeading(spikePos, spikePos.heading)
                .afterTime(0, () -> { // pune pixelul
                    OuttakeAutoCommand outtakeAutoCommand = new OuttakeAutoCommand(robot.intake);
                    outtakeAutoCommand.schedule();
                })
                .waitSeconds(1.6) // iese pixelul
                .splineToLinearHeading(new Pose2d(5, 0, Math.toRadians(-90)), Math.toRadians(-135))
                .waitSeconds(0.5)
                .lineToY(56)
                .waitSeconds(1)
                .afterTime(0, () -> {(new LowExtendCommand(robot.lift, robot.arm, 1300)).schedule();})
                .setReversed(true)
                .splineToLinearHeading(backdropPos, Math.toRadians(-90), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 20;
                    }
                })
                .afterTime(0.4, robot.deposit::deposit)
                .waitSeconds(1.4)
                .afterTime(0, robot.deposit::stop)
                .afterTime(0, () -> retractCommand.schedule())
                .setReversed(false)
                .splineToLinearHeading(StackSidePositions.blueParkPos, Math.toRadians(-45))
                .build();
    }
}
