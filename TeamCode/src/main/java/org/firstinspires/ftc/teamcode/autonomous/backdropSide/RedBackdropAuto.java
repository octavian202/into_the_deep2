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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name = "red backdrop", group = ".")
public class RedBackdropAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(0, 0, 0);

        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startPose);
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap, lift::getPosition);
        Intake intake = new Intake(hardwareMap);
        Deposit deposit = new Deposit(hardwareMap);

        lift.resetEncoder();

        AutoControlCommand autoControlCommand = new AutoControlCommand(lift);
        lift.setDefaultCommand(autoControlCommand);

        RetractCommand retractCommand = new RetractCommand(lift, arm);


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
        ThresholdPipeline pipeline = new ThresholdPipeline(telemetry, "red");
        camera.setPipeline(pipeline);

        int spikePosition = 3;

        while (!isStopRequested() && !isStarted()) {
            spikePosition = pipeline.getPosition();
            telemetry.addData("spike pos", spikePosition);
            telemetry.update();
        }


        waitForStart();

        Pose2d spikePos = BackdropSidePositions.redSpikePos[spikePosition - 1];
        Pose2d backdropPos = BackdropSidePositions.redBackdropPos[spikePosition - 1];


        Action driveAction = mecanumDrive.actionBuilder(startPose)
                // merge la spike
                .splineToLinearHeading(spikePos, spikePos.heading)
                .afterTime(0, () -> { // pune pixelul
                    OuttakeAutoCommand outtakeAutoCommand = new OuttakeAutoCommand(intake);
                    outtakeAutoCommand.schedule();
                })
                .waitSeconds(1.6) // iese pixelul
                .afterTime(0.2, () -> { // ridica bratul
                    (new LowExtendCommand(lift, arm, 1400)).schedule();
                })
                .setReversed(true) // merge la backdrop
                .splineToLinearHeading(backdropPos, backdropPos.heading, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 30;
                    }
                })
                .afterTime(0.4, deposit::deposit) // depoziteaza, sta putin coboara bratul
                .waitSeconds(1.4)
                .afterTime(0d, deposit::stop)
                .afterTime(0d, () -> {
                    retractCommand.schedule();
                })
                .waitSeconds(1)
//                  .splineToLinearHeading(BackdropSidePositions.parkInsidePos, -Math.PI / 3) // parcare in interior
                .splineToLinearHeading(BackdropSidePositions.redParkOutsidePos, Math.PI / 3) // parcare in exterior
                .waitSeconds(0.5)
                .lineToY(BackdropSidePositions.redParkOutsidePos.position.y - 10)
                .build();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            driveAction.run(new TelemetryPacket());
        }

    }
}
