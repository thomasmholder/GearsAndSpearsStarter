package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends BaseHardware {

    public int liftExtended = 3990;

    private double hangLatchClosed = 0.0;
    private double hangLatchOpened = 1.0;

    public double liftLatchClosed = 0.0;
    public double liftLatchOpened = 1.0;

    public double dumpBucketRetracted = 1.0;
    public double dumpBucketDump = 0.0;

    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean yPressed = false;

    private boolean dumping = false;
    private boolean hangLatched = false;
    private boolean liftLatched = false;

    Telemetry telemetry;

    public Lift(){

    }

    public DcMotorEx liftDrive;

    public Servo hangLatch, liftLatch, dumpBucket;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        liftDrive = (DcMotorEx)hardwareMap.dcMotor.get("lift_drive");

        hangLatch = hardwareMap.servo.get("hang_latch");
        liftLatch = hardwareMap.servo.get("lift_latch");
        dumpBucket = hardwareMap.servo.get("dump_bucket");

        liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setup(){

        liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLatch.setPosition(liftLatchClosed);

        dumpBucket.setPosition(dumpBucketRetracted);
    }

    public void manualControl(Gamepad gamepad){

        hangLatchControl(gamepad);
        liftControl(gamepad);
        dumpControl(gamepad);
        liftLatchControl(gamepad);

    }

    private void hangLatchControl(Gamepad gamepad){

        if(gamepad.a && !aPressed){
            hangLatched = !hangLatched;
            aPressed = true;
        }

        if(!gamepad.a){
            aPressed = false;
        }

        if(hangLatched){
            hangLatch.setPosition(hangLatchClosed);
        }
        else{
            hangLatch.setPosition(hangLatchOpened);
        }

    }

    private void liftControl(Gamepad gamepad){
        if(gamepad.left_bumper){
            liftDrive.setPower(-0.5);
        }
        else if(gamepad.right_bumper){
            liftDrive.setPower(0.5);
        }
        else{
            liftDrive.setPower(0);
        }
    }

    private void dumpControl(Gamepad gamepad){
        if(gamepad.b && !bPressed){
            dumping = !dumping;
            bPressed = true;
        }

        if(!gamepad.b){
            bPressed = false;
        }

        if(dumping){
            dumpBucket.setPosition(dumpBucketDump);
        }
        else{
            dumpBucket.setPosition(dumpBucketRetracted);
        }
    }

    private void liftLatchControl(Gamepad gamepad){
        if(gamepad.y && !yPressed){
            liftLatched = !liftLatched;
            yPressed = true;
        }

        if(!gamepad.y){
            yPressed = false;
        }

        if(liftLatched){
            liftLatch.setPosition(liftLatchClosed);
        }
        else{
            liftLatch.setPosition(liftLatchOpened);
        }
    }

}
