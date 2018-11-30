package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class   DriveTrain extends BaseHardware{

    private static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    public static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    public static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    public static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable

    public static final double     CENTER_SAMPLE_DISTANCE  = 20;
    public static final double     SIDE_SAMPLE_DISTANCE    = 22;
    public static final double     SAMPLE_ANGLE            = 25;


    Telemetry telemetry;

    public DriveTrain(){

    }

    public DcMotor leftDrive, rightDrive;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setup(){

    }

    public void manualDrive(Gamepad gamepad){

        double leftPower;
        double rightPower;

        double drive = -gamepad.left_stick_y;
        double turn  =  gamepad.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


        if(!gamepad.right_bumper) {
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }
        else{
            leftDrive.setPower(leftPower/4);
            rightDrive.setPower(rightPower/4);
        }

    }

    public void setPower(double left, double right){
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    public void setMotorModes(DcMotor.RunMode runmode){
        leftDrive.setMode(runmode);
        rightDrive.setMode(runmode);
    }

}
