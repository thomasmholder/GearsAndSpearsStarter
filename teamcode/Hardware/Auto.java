package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.CENTER_SAMPLE_DISTANCE;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.SAMPLE_ANGLE;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.SIDE_SAMPLE_DISTANCE;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.TURN_SPEED;

public class Auto extends BaseHardware{

    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;

    Robot robot;

    public void init(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opmode, Robot robot){
        this.initialize(hardwareMap, telemetry, opmode, robot);
    }

    private void initialize(HardwareMap ahardwareMap, Telemetry atelemetry, LinearOpMode aopMode, Robot arobot){
        hardwareMap = ahardwareMap;
        telemetry = atelemetry;
        opMode = aopMode;
        robot = arobot;
    }


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *  @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
 *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (this.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * robot.driveTrain.COUNTS_PER_INCH);
            newLeftTarget = robot.driveTrain.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.driveTrain.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.driveTrain.leftDrive.setTargetPosition(newLeftTarget);
            robot.driveTrain.rightDrive.setTargetPosition(newRightTarget);

            robot.driveTrain.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveTrain.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.driveTrain.leftDrive.setPower(speed);
            robot.driveTrain.rightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (this.opMode.opModeIsActive() &&
                    (robot.driveTrain.leftDrive.isBusy() && robot.driveTrain.rightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, robot.driveTrain.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.driveTrain.leftDrive.setPower(leftSpeed);
                robot.driveTrain.rightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.driveTrain.leftDrive.getCurrentPosition(),
                        robot.driveTrain.rightDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.driveTrain.leftDrive.setPower(0);
            robot.driveTrain.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.driveTrain.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.driveTrain.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (this.opMode.opModeIsActive() && !onHeading(speed, angle, robot.driveTrain.P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of tim
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (this.opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, robot.driveTrain.P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.driveTrain.leftDrive.setPower(0);
        robot.driveTrain.rightDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     *
     */
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= robot.driveTrain.HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.driveTrain.leftDrive.setPower(leftSpeed);
        robot.driveTrain.rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getGyroangle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     *
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void sample(){

        gyroTurn(TURN_SPEED, SAMPLE_ANGLE);

        sleep(500);

        if(robot.vision.detector.getAligned()){
            gyroDrive(DRIVE_SPEED, SIDE_SAMPLE_DISTANCE,SAMPLE_ANGLE);
            gyroDrive(DRIVE_SPEED, -SIDE_SAMPLE_DISTANCE, SAMPLE_ANGLE);
            return;
        }

        gyroTurn(TURN_SPEED, -SAMPLE_ANGLE);

        sleep(500);

        if(robot.vision.detector.getAligned()){
            gyroDrive(DRIVE_SPEED, SIDE_SAMPLE_DISTANCE,-SAMPLE_ANGLE);
            gyroDrive(DRIVE_SPEED, -SIDE_SAMPLE_DISTANCE, -SAMPLE_ANGLE);
            return;
        }

        gyroTurn(TURN_SPEED, 0);

        sleep(500);

        if(robot.vision.detector.getAligned()){
            gyroDrive(DRIVE_SPEED, CENTER_SAMPLE_DISTANCE,0);
            gyroDrive(DRIVE_SPEED, -CENTER_SAMPLE_DISTANCE, 0);
            return;
        }

    }

    public void land(){

        robot.lift.liftDrive.setMode(RUN_USING_ENCODER);
        robot.lift.liftDrive.setPower(-0.75);


        sleep(500);

        robot.lift.liftLatch.setPosition(1.0);

        sleep(500);

        robot.lift.liftDrive.setMode(RUN_TO_POSITION);

        robot.lift.liftDrive.setTargetPosition(robot.lift.liftExtended);
        robot.lift.liftDrive.setPower(0.5);

        while(robot.lift.liftDrive.isBusy() && this.opMode.opModeIsActive()) {
            robot.lift.liftDrive.setPower(0.5);
        }

        robot.lift.hangLatch.setPosition(1.0);

        robot.lift.dumpBucket.setPosition(0.5);

        sleep(500);

        //Sample
        gyroHold(TURN_SPEED,0,0.5);
        gyroDrive(DRIVE_SPEED, 4, 0);
        robot.lift.liftDrive.setTargetPosition(0);
        robot.lift.liftDrive.setPower(0.5);
    }




}
