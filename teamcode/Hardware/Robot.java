package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public DriveTrain driveTrain = new DriveTrain();
    public Gyro gyro = new Gyro();
    public Accumulator acc = new Accumulator();
    public Lift lift = new Lift();
    public Vision vision = new Vision();
    public Auto auto = new Auto();

    public Robot(){

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        driveTrain.init(hardwareMap, telemetry);
        gyro.init(hardwareMap, telemetry);
        acc.init(hardwareMap, telemetry);
        lift.init(hardwareMap, telemetry);
    }

    public void setup(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opmode){
        vision.init(hardwareMap, telemetry);

        driveTrain.setup();
        acc.setup();
        gyro.setup();
        lift.setup();
        vision.setup();

        auto.init(hardwareMap, telemetry, opmode, this);
    }

}
