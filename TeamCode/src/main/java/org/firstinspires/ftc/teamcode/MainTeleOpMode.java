package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainTeleopOpMode")
public class MainTeleOpMode extends LinearOpMode{
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor intake;
    DcMotor launch;

    Servo loader;
    double servoPosition;
    static final double MAX_POSITION  = 0; //TODO: SET
    static final double MIN_POSITION = 1; //TODO: SET

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loadDelay = new ElapsedTime();
    boolean infiniteLoop = false;

    @Override public void runOpMode() {
        initializeProgram();
        runProgram();
    }


    private void initializeProgram(){
        initializeMotors();
        waitForStart();
        runProgram();
    }

    private void runProgram() {
        while (opModeIsActive()) {

            //drive strafe rotation
            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotation = -gamepad1.right_stick_x;

            double[] speeds = {
                    drive + strafe + rotation, //FL
                    drive - strafe - rotation, //FR
                    drive - strafe + rotation, //BL
                    drive + strafe - rotation, //BR
            };

            double max = 0;
            for(double speed:speeds) {
                if (max < Math.abs(speed) ) {
                    max = Math.abs(speed);
                }
            }



            if (max > 1) {
                for(int i = 0; i < speeds.length; i++) {
                    speeds[i] /= max;
                }
            }



            if (gamepad1.x) intake.setPower(0.58);
            else intake.setPower(0);

            if(gamepad1.y) {
                servoPosition = 0.8;
                loader.setPosition(servoPosition);
            }
             else {
                 servoPosition = 0.55;
                loader.setPosition(servoPosition);
            }
            if (gamepad1.right_trigger == 1) launch.setPower(1);
            else launch.setPower(0);


            telemetry.addData("servo", "%.5f", loader.getPosition());
            
            //this is the bottom of the code, don't write new code below this point
            motorLF.setPower(speeds[0]);
            motorRF.setPower(speeds[1]);
            motorLB.setPower(speeds[2]);
            motorRB.setPower(speeds[3]);
            loader.setPosition(servoPosition);
            telemetry.update();
        }
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
        intake = hardwareMap.get(DcMotor.class, "intake");
        loader = hardwareMap.get(Servo.class, "loader");
        launch = hardwareMap.get(DcMotor.class, "launch");
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.REVERSE);
        launch.setDirection(DcMotor.Direction.REVERSE);

        servoPosition = .55; //TODO: change this to a consistent number
    }
}
