package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class GridPipeline extends OpenCvPipeline {


    // region 1 (400, 50), (600, 250)
    // region 2 (60, 20), (220, 170)

    Mat gridMat;
    int width = 1250, height = 700;
    @Override
    public Mat processFrame(Mat input) {

        for (int j = 50; j <= height; j += 50) {
            for (int i = 50; i <= width; i += 50) {
                Imgproc.rectangle(
                        input,
                        new Point(i, j),
                        new Point(i, j),
                        new Scalar(255, 255, 255),
                        2
                );
            }
        }


        return input;
    }
}
