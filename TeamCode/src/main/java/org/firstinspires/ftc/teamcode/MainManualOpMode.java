/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="Main Manual OpMode")
public class MainManualOpMode extends LinearOpMode {
    private static final int CYCLE_MS = 50;
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    Gamepad gamepad;
    @Override public void runOpMode() {
        initializeProgram();
        runProgram();
        shutdownProgram();
    }
    private void runProgram() {
        while (shouldContinue()) mainLoop();
    }
    private void mainLoop() {
        moveRobot();
        finalizeLoopStage();
    }
    private void initializeProgram() {
        initializeMotors();
        initializeSensors();
        beginMainProgram();
    }
    private void moveRobot() {
        if(gamepad.dpad_up) {
            beginLinearMotion(1);
        } else if (gamepad.dpad_down) {
            beginLinearMotion(1);
        } else if (gamepad.dpad_left) {
            beginRotation(1);
        } else if (gamepad.dpad_right) {
            beginRotation(-1);
        } else {
            stopRobot();
        }
    }
    private void initializeSensors() {
        gamepad = hardwareMap.get(Gamepad.class, "Gamepad 1");
    }
    private boolean shouldContinue() {
        return !isStopRequested() && opModeIsActive();
    }
    private void finalizeLoopStage() {
        telemetry.update();
        sleep(CYCLE_MS);
        idle();
    }
    private void beginMainProgram() {
        waitForStart();
    }
    private void initializeMotors() {
        motorLF = hardwareMap.get(DcMotor.class, "lf_drive");
        motorLB = hardwareMap.get(DcMotor.class, "lb_drive");
        motorRF = hardwareMap.get(DcMotor.class, "rf_drive");
        motorRB = hardwareMap.get(DcMotor.class, "rb_drive");
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
    private void setMotors(int frontBack, int rightLeft, double amount) {
        boolean isPositive = amount >= 0;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        if(!isPositive) direction = DcMotorSimple.Direction.REVERSE;
        double correctedAmount = Math.abs(amount);
        DcMotor[] motors = getMotors(frontBack, rightLeft);
        for(DcMotor motor : motors) {
            motor.setDirection(direction);
            motor.setPower(correctedAmount);
        }
    }
    private void beginLinearMotion(double power) {
        setMotors(0, 0, power);
    }
    private void beginRotation(double power) {
        setMotors(0, 1, power);
        setMotors(0, -1, -power);
    }
    private void stopRobot()
    {
        setMotors(0, 0, 0);
    }
    private boolean stopIfZero(double distance) {
        if(distance == 0.0) {
            stopRobot();
            return true;
        }
        return false;
    }
    private double getPowerFromDistance(double distance) {
        double power = 1;
        if(distance < 0) power = -1;
        return power;
    }
    private void shutdownProgram() {
        motorLF.setPower(0);
        motorLB.setPower(0);
        motorRF.setPower(0);
        motorRB.setPower(0);
    }
}
