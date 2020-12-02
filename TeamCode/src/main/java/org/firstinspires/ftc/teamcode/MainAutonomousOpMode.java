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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="Main Autonomous OpMode")
public class MainAutonomousOpMode extends LinearOpMode {
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private VuforiaLocalizer.Parameters cameraParameters = null;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final int CYCLE_MS = 50;
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    private static final String VUFORIA_KEY = "AX3wJCr/////AAABmXqqKIuTJkMatbvyrDkSphp5h9VYO42DWDiKLucp30xEvP98y9OIRKGNszB+EpBJ4cPqww5PWGd6BPOl7kHDKajvlJClovU1+L4gNxeM0pvROvjulRueLD7JCqzM7yWina4gLO1YTeWIUGqF1v1Qh34137m65frOnjbPcBgQk2O4ky740K3T/SM541xP4ALr3FSvyl2xQD1EBu2xI49XL2bLtsztTuUUUmyS07lvmYNt4kH1+108Bqvca+GYnufjYs8mlDG4qYSF6UhZyGHhjskblJWKaNQT2Lph3JgxMXfcaV40/qkq9C52GVbIk/QLEDRnKq3Ezce13ZM+GK+YItEAYXJ+hUZHTUIbVedF8uZ4";
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    private VuforiaLocalizer vuforia = null;
    private static final float[][] trackableTransforms = {
            { halfField, quadField, mmTargetHeight, 90, 0, -90 },
            { halfField, -quadField, mmTargetHeight, 90, 0, -90 },
            { 0, -halfField, mmTargetHeight, 90, 0, 180 },
            { 0, halfField, mmTargetHeight, 90, 0, 0 },
            { -halfField, 0, mmTargetHeight, 90, 0, 90 }
    };
    private static final String[] trackableNames = {
            "Blue Tower Goal Target",
            "Red Tower Goal Target",
            "Red Alliance Target",
            "Blue Alliance Target",
            "Front Wall Target"
    };
    private static final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;
    private static final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;
    private static final float CAMERA_LEFT_DISPLACEMENT     = 0;
    WebcamName webcamName = null;
    VectorF translation = null;
    Orientation rotation = null;
    VuforiaTrackables targetsUltimateGoal = null;
    @Override public void runOpMode() {
        initializeProgram();
        runProgram();
        shutdownProgram();
    }
    private void runProgram() {
        while (shouldContinue()) mainLoop();
    }
    private void mainLoop() {
        updateLocations();
        finalizeLoopStage();
    }
    private void initializeProgram() {
        initializeVuforia();
        initializeTargets();
        initializeMotors();
        beginMainProgram();
    }
    private boolean shouldContinue() {
        return !isStopRequested() && opModeIsActive();
    }
    private void initializeVuforia() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cameraParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        cameraParameters.vuforiaLicenseKey = VUFORIA_KEY;
        cameraParameters.cameraName = webcamName;
        cameraParameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(cameraParameters);
    }
    private void initializeTargets() {
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        allTrackables.addAll(targetsUltimateGoal);
        for(int i = 0; i < trackableNames.length; i++) {
            VuforiaTrackable trackable = allTrackables.get(i);
            trackable.setName(trackableNames[i]);
            transform(trackable, trackableTransforms[i]);
        }
    }
    private void finalizeLoopStage() {
        telemetry.update();
        sleep(CYCLE_MS);
        idle();
    }
    private boolean updateLocations() {
        getCameraLocation();
        return getRobotLocation();
    }
    private void beginMainProgram() {
        getCameraLocation();
        waitForStart();
        targetsUltimateGoal.activate();
    }
    private void initializeMotors() {
        motorLF = hardwareMap.get(DcMotor.class, "lf_drive");
        motorLB = hardwareMap.get(DcMotor.class, "lb_drive");
        motorRF = hardwareMap.get(DcMotor.class, "rf_drive");
        motorRB = hardwareMap.get(DcMotor.class, "rb_drive");
    }
    private void getCameraLocation() {
        float phoneYRotate = 0;
        float phoneXRotate = 0;
        float phoneZRotate = 0;
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, cameraParameters.cameraDirection);
        }
    }
    private boolean getRobotLocation() {
        ArrayList<OpenGLMatrix> allVisible = new ArrayList<OpenGLMatrix>();
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getRobotLocation();
                allVisible.add(robotLocationTransform);
            }
        }
        if (!allVisible.isEmpty()) {
            translation = new VectorF(0, 0, 0);
            for (OpenGLMatrix visible : allVisible) {
                translation.add(visible.getTranslation());
            }
            translation.multiply(1f / allVisible.size());
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            VectorF rotationTotal = new VectorF(0, 0, 0);
            for (OpenGLMatrix visible : allVisible) {
                Orientation singleRotation = Orientation.getOrientation(visible, EXTRINSIC, XYZ, DEGREES);
                rotationTotal.add(new VectorF(singleRotation.firstAngle, singleRotation.secondAngle, singleRotation.thirdAngle));
            }
            rotation = new Orientation(EXTRINSIC, XYZ, DEGREES,
                    rotationTotal.get(0) / allVisible.size(),
                    rotationTotal.get(1) / allVisible.size(),
                    rotationTotal.get(2) / allVisible.size(), System.nanoTime());
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            return true;
        }
        telemetry.addData("Visible Target", "none");
        return false;
    }
    private void shutdownProgram() {
        motorLF.setPower(0);
        motorLB.setPower(0);
        motorRF.setPower(0);
        motorRB.setPower(0);
        targetsUltimateGoal.deactivate();
    }
    private void transform(VuforiaTrackable trackable, float locationX, float locationY, float locationZ, float angle1, float angle2, float angle3)
    {
        trackable.setLocation(OpenGLMatrix
                .translation(locationX, locationY, locationZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, angle1, angle2, angle3)));
    }
    private void transform(VuforiaTrackable trackable, float[] values)
    {
        transform(trackable, values[0], values[1], values[2], values[3], values[4], values[5]);
    }
}