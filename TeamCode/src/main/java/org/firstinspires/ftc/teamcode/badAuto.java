package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "bad auto")
public class badAuto extends LinearOpMode {
    //motors
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor intake;
    //DcMotor launch;
    DcMotor[] motors = new DcMotor[4];

    Servo loader;
    double servoPosition;
    static final double MAX_POSITION  = 0; //TODO: SET
    static final double MIN_POSITION = 1; //TODO: SET

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loadDelay = new ElapsedTime();

    static final double COUNTS_PER_CM = 1000 / 28.25; //cm.
    @Override
    public void runOpMode() throws InterruptedException {
        runSetup();
        waitForStart();

        driveTicks(1, 1000, 0);
        driveTicks(1, -1000, -1000);
        driveTicks(1, 0, 1000);
    }

    void runSetup() {
        motorLF = hardwareMap.get(DcMotor.class, "lf_drive");
        motorLB = hardwareMap.get(DcMotor.class, "lb_drive");
        motorRF = hardwareMap.get(DcMotor.class, "rf_drive");
        motorRB = hardwareMap.get(DcMotor.class, "rb_drive");
        motors[0] = motorLF;
        motors[1] = motorLB;
        motors[2] = motorRF;
        motors[3] = motorRB;
        intake = hardwareMap.get(DcMotor.class, "intake");
        loader = hardwareMap.get(Servo.class, "loader");
        //launch = hardwareMap.get(DcMotor.class, "launch");
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.REVERSE);

        runtime.reset();

    }

    void driveTicks(double speed, int forwardTicks, int horizontalTicks) {

        int[] targets = {
                forwardTicks + horizontalTicks, //lfh
                forwardTicks - horizontalTicks, //lb
                forwardTicks - horizontalTicks, //rf
                forwardTicks + horizontalTicks, //rb
        };

        double[] speeds = new double[4];

        //find the target with greatest magnitude
        int maxTarget = 0;
        for(int i = 0; i < motors.length; i++) {
            if (maxTarget < Math.abs(targets[i])) {
                maxTarget = Math.abs(targets[i]);
            }
        }

        for(int i = 0; i < motors.length; i++) {
            speeds[i] = ((double) targets[i] / maxTarget) * speed; //the fastest wheel should be going at the speed "speed"
        }

        for(int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(targets[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(speeds[i]);
        }

        while(opModeIsActive() && (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy())) { //wait for stuff to done
        }

        for(DcMotor motor: motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }
}
