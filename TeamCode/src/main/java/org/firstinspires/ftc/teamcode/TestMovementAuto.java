package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestMovementAuto")
public class TestMovementAuto extends LinearOpMode {
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    boolean infiniteLoop = false;
    private ElapsedTime runtime = new ElapsedTime();
    @Override public void runOpMode() {
        initializeProgram();
        waitForStart();
        runProgram();
    }


    private void initializeProgram(){
        initializeMotors();
        runProgram();
    }

    private void runProgram() {

        //lefts are positive, rights are negative

        // Step 1:  Drive forward for 3 seconds
        motorLB.setPower(.6);
        motorLF.setPower(.6);
        motorRB.setPower(-.6);
        motorRF.setPower(-.6);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //halt, dumbass
        motorLB.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorRF.setPower(0);

    }

    private DcMotor[] getMotors(int frontBack, int rightLeft)
    {
        DcMotor[][][] returnArray = new DcMotor[][][] {
                new DcMotor[][] {
                        new DcMotor[] {
                                motorLB
                        },
                        new DcMotor[] {
                                motorRB, motorLB
                        },
                        new DcMotor[] {
                                motorRB
                        }
                },
                new DcMotor[][] {
                        new DcMotor[] {
                                motorLF, motorLB
                        },
                        new DcMotor[] {
                                motorRF, motorLF, motorRB, motorLB
                        },
                        new DcMotor[] {
                                motorRF, motorRB
                        }
                },
                new DcMotor[][] {
                        new DcMotor[] {
                                motorLF
                        },
                        new DcMotor[] {
                                motorRF, motorLF
                        },
                        new DcMotor[] {
                                motorRF
                        }
                }
        };
        return returnArray[frontBack + 1][rightLeft + 1];
    }

    private void initializeMotors() {
        motorLF = hardwareMap.get(DcMotor.class, "lf_drive");
        motorLB = hardwareMap.get(DcMotor.class, "lb_drive");
        motorRF = hardwareMap.get(DcMotor.class, "rf_drive");
        motorRB = hardwareMap.get(DcMotor.class, "rb_drive");
    }

    public void moveForward(double power){
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        if (power<0) direction = DcMotorSimple.Direction.REVERSE;
        double correctedPower = Math.abs(power);
        motorLF.setDirection(direction);
        motorLB.setDirection(direction);
        motorRF.setDirection(direction);
        motorRB.setDirection(direction);

        motorLB.setPower(correctedPower);
        motorLF.setPower(correctedPower);
        motorRB.setPower(correctedPower);
        motorRF.setPower(correctedPower);
    }

    public void strafe(double power){
        DcMotorSimple.Direction direction1 = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction direction2 = DcMotorSimple.Direction.REVERSE;
        if (power<0) {
            direction1 = DcMotorSimple.Direction.REVERSE;
            direction2 = DcMotorSimple.Direction.FORWARD;
        }
        double correctedPower = Math.abs(power);
        motorLF.setDirection(direction2);
        motorLB.setDirection(direction1);
        motorRF.setDirection(direction1);
        motorRB.setDirection(direction2);

        motorLB.setPower(correctedPower);
        motorLF.setPower(correctedPower);
        motorRB.setPower(correctedPower);
        motorRF.setPower(correctedPower);


    }
}

