package org.firstinspires.ftc.teamcode;



import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name = "WebcamAuton")
public class WebcamAuton extends LinearOpMode
{


    private static final String Vuforia_Key = "ATOQy0T/////AAABmQ/SASRSuUzAugTNY1JUz6NSX/K0IyvPgOTMGlPcjAUyxoQlqULPX1jcW4C4fMzALWwznPmVdS4QFyFERfGevgAJPX1U8c6c1wfPTrhaqwhhoG0SBo/8b6iGaeweb65NN1Xu7PG+LHieN8rr339wsfsGROM9TW3oRp6uYdDFq30aM9tAIadifcbtQq9XUSdUyzF7Owgr8QjIbAw57OYIb6Bwl+7tenxUVSM+pYEkZyVdbCWAWEHAeu10tX7qGXmPEENNiKfnTT/TvGCsjTVK0Xa536mx70V74J/wBadTw32md6QMKjQmWWtCqyYnYZenYws42NGZzXsD/cLU+IbXOIJBaohDU8SaPr7mienrXGgc";



    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;


    @Override
    public void runOpMode() throws InterruptedException
    {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = Vuforia_Key;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        waitForStart();
        vuforia.enableConvertFrameToBitmap();
        vuforia.setFrameQueueCapacity(1);




    }
    public Bitmap getBitmap(){
        Bitmap bitmap;
        VuforiaLocalizer.CloseableFrame frame;

        try {
            frame = vuforia.getFrameQueue().take();
        }catch (Exception e) {
            throw new Warning("couldn't find vuforia frame");
        }

        bitmap = vuforia.convertFrameToBitmap(frame);
        return bitmap;
    }

}