package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoScorePixel;
import org.firstinspires.ftc.teamcode.commands.Intake.StartIntake;
import org.firstinspires.ftc.teamcode.commands.Intake.StartIntakeForStack;
import org.firstinspires.ftc.teamcode.commands.Intake.StopIntake;
import org.firstinspires.ftc.teamcode.commands.LowExtendCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SpikeDetectionCamera;

import java.util.List;

public class AutoBase extends LinearOpMode {

    protected Robot robot;
    protected SpikeDetectionCamera spikeDetectionCamera;
    protected String color = "red";
    protected LowExtendCommand lowExtendCommand, midExtendCommand;
    protected RetractCommand retractCommand;
    protected AutoScorePixel autoScoreCommand;
    protected VelConstraint lowVelConstraint = new VelConstraint() {
        @Override
        public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
            return 30;
        }
    };
    protected VelConstraint midVelConstraint = new VelConstraint() {
        @Override
        public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
            return 40;
        }
    };

    protected VelConstraint highVelConstraint = new VelConstraint() {
        @Override
        public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
            return 50;
        }
    };

    protected TelemetryPacket telemetryPacket = new TelemetryPacket();
    protected Pose2d startPose = new Pose2d(0, 0, 0);
    protected int spikePosition = 3;
    protected Action driveAction;
    protected StartIntake startIntakeCommand;
    protected StopIntake stopIntakeCommand;
    protected StartIntakeForStack startIntakeForStackCommand;

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, Robot.OpModeType.Auto);

        setColor();
        spikeDetectionCamera = new SpikeDetectionCamera(hardwareMap, color, telemetry);

        retractCommand = new RetractCommand(robot.lift, robot.arm);
        lowExtendCommand = new LowExtendCommand(robot.lift, robot.arm, 1150);
        midExtendCommand = new LowExtendCommand(robot.lift, robot.arm, 1400);

        autoScoreCommand = new AutoScorePixel(robot.lift, robot.arm, robot.deposit);

        startIntakeCommand = new StartIntake(robot.intake, robot.deposit);
        stopIntakeCommand = new StopIntake(robot.intake, robot.deposit);
        startIntakeForStackCommand = new StartIntakeForStack(robot.intake, robot.deposit, robot.arm);

        while (!isStopRequested() && !isStarted()) {
            spikePosition = spikeDetectionCamera.getDetection();
        }

        waitForStart();
        spikeDetectionCamera.close();

        afterInit();

        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            driveAction.run(telemetryPacket);
        }
    }

    protected void afterInit() {}
    protected void setColor() {}
}
