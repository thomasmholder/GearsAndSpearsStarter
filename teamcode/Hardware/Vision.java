package org.firstinspires.ftc.teamcode.Hardware;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.ArrayList;
import java.util.List;

// Vuforia variables


public class Vision extends BaseHardware {

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    // DogeCV detector
    public GoldAlignDetector detector;


    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Vuforia licence key
        parameters.vuforiaLicenseKey = "AasL0yz/////AAABmUXduuOCnUs3rbkNa5olSJck+3OMQH4EkfgqX+kR11uOTJnoaq+7XtGOEoJe29YGQ4B0yRUKt+Cj7fiQJEb3iSrGHJOVOpq2XIFNL+W5QlONzDQzrCjDnJm8ZrXy3AniA14eTXNZiPUjXBR1WTE7CcVLFjRlMUwvIm46hS1ZnRncjBhtKlOA0AsfNmubGCVcX06dOMeI42fuCrV+eI0HxYscVk9WK6yKLSTaS44Yh9I6tsQvRFs19gW5xyiUOozpU2pRT52gGnoi85MnJO3SpBHrIKL17aYlnAFKZPBUGdM7hMLX15VgLiIMW72bfqj9YI4p7MwJFps0TbkjKZng0fJtaUiuGkThvSDOKz0/QQOt";
        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = webcamName;

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Initialize the detector
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;
        detector.alignSize = 300;


        // Set the detector
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

    }

    public void setup(){
        this.detector.enable();
    }

}
