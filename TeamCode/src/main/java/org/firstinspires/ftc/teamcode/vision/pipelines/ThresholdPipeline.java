package org.firstinspires.ftc.teamcode.vision.pipelines;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class ThresholdPipeline extends OpenCvPipeline {

    TeamPropPosition teamPropPosition = TeamPropPosition.LEFT;
    Color color;
    public static int RED_RECOGNITION_TRESHOLD = 20;
    public static int BLUE_RECOGNITION_TRESHOLD = 20;
    private int threshold = 50;

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */

    public static Scalar blueLower = new Scalar(130, 80, 0);
    public static Scalar blueUpper = new Scalar(190, 255, 255);

    public static Scalar redLower = new Scalar(160, 90, 0);
    public static Scalar redUpper = new Scalar(210, 255, 255);

    public static Scalar lower;
    public static Scalar upper;

    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */
    Telemetry telemetry;

    private Mat hsvMat         = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();

    Mat region1, region2;

    // region 2 (70, 70), (220, 210)
    // region 3 (350, 80), (550, 270)
    Point zone1UpLeft = new Point(240, 380), zone1DownRight = new Point(480, 600);
    Point zone2UpLeft = new Point(800, 350), zone2DownRight = new Point(1000, 520);

    public ThresholdPipeline(Telemetry telemetry, String color) {
        
        switch (color) {
            case "red":
                this.color = Color.RED;
                upper = redUpper;
                lower = redLower;
                threshold = RED_RECOGNITION_TRESHOLD;
                break;

            case "blue":
                this.color = Color.BLUE;
                upper = blueUpper;
                lower = blueLower;
                threshold = BLUE_RECOGNITION_TRESHOLD;
                break;
            default:
                break;
        }
        
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame) {
        region1 = firstFrame.submat(new Rect(zone1UpLeft, zone1DownRight));
        region2 = firstFrame.submat(new Rect(zone2UpLeft, zone2DownRight));
    }

    @Override
    public Mat processFrame(Mat input) {

        /*
         * Converts our input mat from RGB / BGR to HSV.
         * EOCV ALWAYS returns RGB mats, so you'd
         * always convert from RGB to the color
         * space you want to use.
         *
         * Takes our "input" mat as an input, and outputs
         * to a separate Mat buffer "hsvMat"
         * Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);
         */

        switch (color) {
            case RED:
                Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_BGR2HSV_FULL);
                break;
            case BLUE:
                Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
                break;

        }

        /*
         * This is where our thresholding actually happens.
         * Takes our "hsv" as input and outputs a "binary"
         * Mat to "binaryMat" of the same size as our input.
         * "Discards" all the pixels outside the bounds specified
         * by the scalars above (and modifiable with EOCV-Sim's
         * live variable tuner.)
         *
         * Binary meaning that we have either a 0 or 255 value
         * for every pixel.
         *
         * 0 represents our pixels that were outside the bounds
         * 255 represents our pixels that are inside the bounds
         */
        Core.inRange(hsvMat, lower, upper, binaryMat);

        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();

        /*
         * Now, with our binary Mat, we perform a "bitwise and"
         * to our input image, meaning that we will perform a mask
         * which will include the pixels from our input Mat which
         * are "255" in our binary Mat (meaning that they're inside
         * the range) and will discard any other pixel outside the
         * range (RGB 0, 0, 0. All discarded pixels will be black)
         */
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        /*
         * The Mat returned from this method is the
         * one displayed on the viewport.
         *
         * To visualize our threshold, we'll return
         * the "masked input mat" which shows the
         * pixel from the input Mat that were inside
         * the threshold range.
         */

        Core.copyTo(maskedInputMat, input, input);


        int avg1, avg2;

        switch (color) {
            case RED:
                avg1 = (int) Core.mean(region1).val[0];
                avg2 = (int) Core.mean(region2).val[0];
                break;
            case BLUE:
                avg1 = (int) Core.mean(region1).val[2];
                avg2 = (int) Core.mean(region2).val[2];
                break;
            default:
                avg1 = 0;
                avg2 = 0;
                break;
        }


        telemetry.addData("avg1", avg1);
        telemetry.addData("avg2", avg2);
        telemetry.update();

        Imgproc.rectangle(
                input,
                zone1UpLeft,
                zone1DownRight,
                new Scalar(255, 255, 255),
                2
        );
        Imgproc.rectangle(
                input,
                zone2UpLeft,
                zone2DownRight,
                new Scalar(255, 255, 255),
                2
        );


        int max = Math.max(avg1, avg2);

        if (max < threshold) { // pozitia care nu e vazuta
            teamPropPosition = TeamPropPosition.RIGHT;
        } else if (max == avg2) {
            teamPropPosition = TeamPropPosition.MIDDLE;
            Imgproc.rectangle(
                    input,
                    zone2UpLeft,
                    zone2DownRight,
                    new Scalar(0, 255, 0),
                    2
            );
        } else if (max == avg1) {
            teamPropPosition = TeamPropPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    zone1UpLeft,
                    zone1DownRight,
                    new Scalar(0, 255, 0),
                    2
            );
        }


        return input;
    }

    public TeamPropPosition getTeamPropPosition() {
        return teamPropPosition;
    }
    public int getPosition() {
        switch (teamPropPosition) {
            case LEFT:
                return 1;
            case MIDDLE:
                return 2;
            case RIGHT:
                return 3;
            default:
                return 0;
        }
    }


    enum TeamPropPosition {
        LEFT,
        MIDDLE,
        RIGHT,
    }

    enum Color {
        RED,
        BLUE
    }

}