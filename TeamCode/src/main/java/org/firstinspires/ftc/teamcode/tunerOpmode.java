package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name="tunerOpmode")
@Config
public class tunerOpmode extends LinearOpMode {
    private Elevator2 elevator;
    private DcMotor left = hardwareMap.get(DcMotor.class,"leftLift"); //left and right are when viewed from the back
    private DcMotor right = hardwareMap.get(DcMotor.class, "rightLift");
    public static double p,i,d,f;
    private PIDFCoefficients pidf;
    private double spoolDiameter = 1.3;
    private double ticksPerRev = 145.6;
    private double maxExtension = 27;
    private boolean isLeftReversed = true;
    private boolean isRightReversed = true;
    private int tolerance = 10;
    private RevBlinkinLedDriver blinkin;

    @Override
    public void runOpMode(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
        pidf = new PIDFCoefficients(p,i,d,f);
        elevator = new Elevator2(left,right,pidf,spoolDiameter,ticksPerRev,maxExtension,isLeftReversed,isRightReversed,tolerance,true);
        elevator.setMode(Elevator2.elevState.MANUAL);
        elevator.setVelocity(0.75);
        while (elevator.getCurrentPos() <= 0.75*maxExtension) {
            elevator.update();
            elevator.sendTelemetry();
        }
        elevator.stop();
        elevator.setVelocity(-0.75);
        while (elevator.getCurrentPos() >= 0.5) {
            elevator.update();
        }
        elevator.stop();

    }
}
