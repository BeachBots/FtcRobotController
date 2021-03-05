
    /*
     * Copyright (c) 2019 OpenFTC Team
     *
     * Permission is hereby granted, free of charge, to any person obtaining a copy
     * of this software and associated documentation files (the "Software"), to deal
     * in the Software without restriction, including without limitation the rights
     * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
     * copies of the Software, and to permit persons to whom the Software is
     * furnished to do so, subject to the following conditions:
     *
     * The above copyright notice and this permission notice shall be included in all
     * copies or substantial portions of the Software.
     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
     * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
     * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
     * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
     * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
     * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
     * SOFTWARE.
     */

    package org.firstinspires.ftc.teamcode;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.opencv.core.Core;
    import org.opencv.core.CvType;
    import org.opencv.core.Mat;
    import org.opencv.core.MatOfPoint;
    import org.opencv.core.MatOfPoint2f;
    import org.opencv.core.Point;
    import org.opencv.core.Rect;
    import org.opencv.core.RotatedRect;
    import org.opencv.core.Scalar;
    import org.opencv.imgproc.Imgproc;
    import org.openftc.easyopencv.OpenCvCamera;
    import org.openftc.easyopencv.OpenCvCameraFactory;
    import org.openftc.easyopencv.OpenCvCameraRotation;
    import org.openftc.easyopencv.OpenCvPipeline;

    import java.util.ArrayList;
    import java.util.List;

    import static org.opencv.core.Core.inRange;
    import static org.opencv.core.Core.mean;

@Disabled
@Config
@TeleOp
//Disabled
    public class Webcam extends LinearOpMode {

        public static int VALUE_0A = 0;
        public static int VALUE_0B = 25;
        public static int VALUE_1A = 100;
        public static int VALUE_1B = 255;
        public static int VALUE_2A = 80;
        public static int VALUE_2B = 255;
        public static int val = 1;
        OpenCvCamera webcam;

        double[] bottomRollingAvg = {0, 0, 0};
        double bottomAvg = 0;
        int stack;
        boolean objectDetected = false;

        @Override
        public void runOpMode() {

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

            webcam.setPipeline(new stackPipeline());

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
            });
            FtcDashboard.getInstance().startCameraStream(webcam, 0);

            telemetry.addLine("Waiting for start");

            telemetry.update();

            /*
             * Wait for the user to press start on the Driver Station
             */
            waitForStart();
            while (opModeIsActive()) {

                /*
                 * Send some stats to the telemetry
                 */

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("bottom Avg", bottomAvg);
            telemetry.addData("Stack of:", stack);
            telemetry.update();




                if (gamepad1.a) {

                    webcam.stopStreaming();

                }


                sleep(100);
            }
            webcam.stopStreaming();
        }


        class stackPipeline extends OpenCvPipeline {


            boolean viewportPaused;




            Mat submat;

            Mat greyBottom = new Mat();
            Mat matTest = new Mat();
            Point[] prevRectPoints;
            double distancefromCenter;

            @Override
            public void init(Mat firstFrame)
            {
                submat = firstFrame.submat(0,240, 0, 320);

            }

            @Override
            public Mat processFrame(Mat input) {
                greyBottom = input.submat(100, 240, 60, 320);

                Imgproc.cvtColor(greyBottom, greyBottom, Imgproc.COLOR_RGB2HSV);
                Scalar lowerOrange = new Scalar(VALUE_0A, VALUE_1A, VALUE_2A);
                Scalar upperOrange = new Scalar(VALUE_0B, VALUE_1B, VALUE_2B);
                inRange(greyBottom, lowerOrange, upperOrange, greyBottom);

                List<MatOfPoint> test = new ArrayList<MatOfPoint>();

                Imgproc.findContours(greyBottom, test, matTest, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

                objectDetected = false;

                bottomRollingAvg = new double[]{0, 0, 0};
                bottomAvg = (int) Core.mean(greyBottom).val[0];

                if (bottomAvg < 5) {
                    stack = 0;
                } else if ((bottomAvg > 5) && (bottomAvg < 12)) {
                    stack = 1;
                } else if (bottomAvg > 12) {
                    stack = 4;
                }
                if (val == 0) {
                    return input;
                } else {
                    return greyBottom;
                }

            }

            //DON'T NEED THIS

            @Override
            public void onViewportTapped() {
                /*
                 * The viewport (if one was specified in the constructor) can also be dynamically "paused"
                 * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
                 * when you need your vision pipeline running, but do not require a live preview on the
                 * robot controller screen. For instance, this could be useful if you wish to see the live
                 * camera preview as you are initializing your robot, but you no longer require the live
                 * preview after you have finished your initialization process; pausing the viewport does
                 * not stop running your pipeline.
                 *
                 * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
                 */

                viewportPaused = !viewportPaused;

                if (viewportPaused) {
                    webcam.pauseViewport();
                } else {
                    webcam.resumeViewport();
                }
            }
        }
    }

