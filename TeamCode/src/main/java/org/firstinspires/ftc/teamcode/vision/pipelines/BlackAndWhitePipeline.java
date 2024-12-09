package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlackAndWhitePipeline extends OpenCvPipeline {

    Mat grey = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        return grey;
    }
}
