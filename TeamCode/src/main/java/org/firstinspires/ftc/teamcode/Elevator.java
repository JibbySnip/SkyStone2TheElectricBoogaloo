package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;



class Elevator {

    private DcMotorEx left,right; //when viewed from back
    private double spoolDiameter;
    private PIDFCoefficients coeffs;
    private double currentPosL,currentPosR = 0;
    private double ticksPerRev, maxExtension, desiredVelocity;
    private boolean isLeftReversed,isRightReversed;
    private int tolerance;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private double manualAdjust;



    /**
     * Returns an instance of the elevator class, intended to drive linear slides
     * @param left  the left slide when viewed from the back
     * @param right the right slide when viewed from the back
     * @param coeffs    coefficients for the PID control
     * @param spoolDiameter     diameter of the spool in inches
     * @param ticksPerRev   the ticks per full motor revolution
     * @param maxExtension  the length of the full extension from the top of the lowest stage to the top of the highest stage
     * @param isLeftReversed    whether the extension direction for the left motor is the motor's backwards direction
     * @param isRightReversed   whether the extension direction for the right motor is the motor's backwards direction
     * @param tolerance the desired tolerance of run_to_position() in ticks
     * @see VirtualFourBar
     */
    Elevator(DcMotor left, DcMotor right, PIDFCoefficients coeffs, double spoolDiameter, double ticksPerRev, double maxExtension, boolean isLeftReversed, boolean isRightReversed, int tolerance){
        this.left = (DcMotorEx) left;
        this.right = (DcMotorEx) right;
        this.spoolDiameter = spoolDiameter;
        this.ticksPerRev = ticksPerRev;
        this.isLeftReversed = isLeftReversed;
        this.isRightReversed = isRightReversed;
        this.maxExtension = maxExtension;
        this.tolerance = tolerance;
        this.coeffs = coeffs;
        motorInits();

    }


    private void motorInits(){
        left.setDirection(isLeftReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(isRightReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setVelocityPIDFCoefficients(coeffs.p,coeffs.i,coeffs.d,coeffs.f);
        left.setPositionPIDFCoefficients(0);
        left.setTargetPositionTolerance(tolerance);
        right.setTargetPositionTolerance(tolerance);

    }


    private double ticksToInches(double ticks,boolean isReversed){
        double count = (ticks/ticksPerRev) * spoolDiameter * 2 * Math.PI;
        return isReversed ? -count : count;
    }

    private double inchesToTicks(double inches,boolean isReversed) {
        double count = (inches*ticksPerRev)/(spoolDiameter*2*Math.PI);
        return isReversed ? -count : count;
    }

    /**
     * Updates the PIDF coefficients while the program is running
     * @param coeffs
     */
    void updateCoeffs(PIDFCoefficients coeffs){
        this.coeffs = coeffs;
        motorInits();
    }

    /**
     * Sets the velocity of the slides
     * @param velocity velocity between -1 and 1
     */
    void setVelocity(double velocity){
        desiredVelocity = velocity;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setVelocity(velocity);
        left.setVelocity(velocity);
    }

    /**
     * Updates the current position, and makes sure the slides don't break
     */
    void update() {
        currentPosL = ticksToInches(left.getCurrentPosition(), isLeftReversed);
        currentPosR = ticksToInches(right.getCurrentPosition(), isRightReversed);
        if (Math.abs(currentPosL-currentPosR) > 10) {
            disableLift();
        } else if ((currentPosL >= maxExtension - 0.25 || currentPosR >= maxExtension - 0.25) && (desiredVelocity > 0)) {
            setVelocity(0);
        } else if ((currentPosL <= 0.25 || currentPosR <= 0.25) && desiredVelocity < 0) {
            setVelocity(0);
        }

    }

    /**
     * Runs to a position in inches
     * @param pos the desired position, in inches
     */
    void runToPosition(double pos){
        double adjustL = isLeftReversed ? -manualAdjust : manualAdjust;
        double adjustR = isRightReversed ? -manualAdjust : manualAdjust;

        left.setTargetPosition((int) (inchesToTicks(pos,isLeftReversed) + adjustL));
        right.setTargetPosition((int) (inchesToTicks(pos,isRightReversed) + adjustR));
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Disables the lift system
     */
    void disableLift() {
        stop();
        left.setMotorDisable();
        right.setMotorDisable();
    }

    /**
     * Sends telemetry data to the FTC Dashboard
     */
    void sendTelemetry() {
        TelemetryPacket positions = new TelemetryPacket();
        positions.put("left_position",currentPosL);
        positions.put("right_position",currentPosR);
        positions.put("left_desired_position",ticksToInches(left.getTargetPosition(),isLeftReversed));
        positions.put("right_desired_position",ticksToInches(right.getTargetPosition(),isRightReversed));
        positions.put("left_velocity",left.getVelocity());
        positions.put("right_velocity",right.getVelocity());
        positions.put("desired_velocity",desiredVelocity);
        dashboard.sendTelemetryPacket(positions);
    }

    /**
     * Gives the current position of the slides
     * @return The current position of the slides, in inches
     */
    double getCurrentPos() {
        return currentPosL;
    }

    /**
     * Gives a custom slight increase to the target position
     * @param manualAdjust the increase amount, in inches
     */
    void setManualAdjust(double manualAdjust) {
        this.manualAdjust += manualAdjust;
    }

    /**
     * Gives an increase to manualAdjust of the default size
     */
    void incrManualAdjust() {
        manualAdjust += 0.25;
    }

    /**
     * Gives a decrease to manualAdjust of the default size
     */
    void decrManualAdjust() {
        manualAdjust -= 0.25;
    }
    

    /**
     * Stops the slides
     */
    void stop(){
        setVelocity(0);
    }

    /**
     * Brings the slides to the minimum extension.
     */
    void goToBottom(){
        runToPosition(0);
    }
}
