package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;




public class PositionSampling extends GoldAlignDetector {

    enum goldPosition{
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    goldPosition position;

    private GoldAlignDetector detector;

    public void setupDetector(HardwareMap hardwareMap) {
        detector = new PositionSampling();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
    }

    public void getGoldByPosition() {

        double goldX = detector.getXPosition();

        int leftBound = 200;
        int rightBound = 500;

        if(!detector.isFound()){
            position = goldPosition.UNKNOWN;
        }

        else if(goldX < leftBound){
            position = goldPosition.LEFT;
        }

        else if(goldX > rightBound){
            position = goldPosition.RIGHT;
        }

        else{
            position = goldPosition.CENTER;
        }


        detector.disable();

    }
}


