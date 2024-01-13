package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.LowExtendCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.pipelines.ThresholdPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public class AutoBase extends LinearOpMode {

    protected Robot robot;
    protected int spikePosition = 3;
    protected Action driveAction;
    protected RetractCommand retractCommand;
    protected LowExtendCommand lowExtendCommand;
    protected String color = "blue";

    protected VelConstraint lowVelConstraint = new VelConstraint() {
        @Override
        public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
            return 25;
        }
    };
    protected VelConstraint midVelConstraint = new VelConstraint() {
        @Override
        public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
            return 35;
        }
    };
    protected VelConstraint highVelConstraint = new VelConstraint() {
        @Override
        public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
            return 45;
        }
    };
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Robot robot = new Robot(hardwareMap, Robot.OpModeType.Auto);

        retractCommand = new RetractCommand(robot.lift, robot.arm);
        lowExtendCommand = new LowExtendCommand(robot.lift, robot.arm, 1400);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        OpenCvCamera.AsyncCameraOpenListener cameraOpenListener = new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        };

        camera.openCameraDeviceAsync(cameraOpenListener);
        ThresholdPipeline pipeline = new ThresholdPipeline(telemetry, color);
        camera.setPipeline(pipeline);

        while (!isStopRequested() && !isStarted()) {
            spikePosition = pipeline.getPosition();
            telemetry.addData("spike pos", spikePosition);
            telemetry.update();
        }

        TelemetryPacket telemetryPacket = new TelemetryPacket();

        onStarted();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            if (driveAction != null) {
                driveAction.run(telemetryPacket);
            }
        }

    }

    public void onStarted() {}

}
