package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "BaseProgram_AprilTag", group = "LinearOpMode")
public class CameraCode_v1 extends LinearOpMode {

    // Drive motors
    private DcMotor frontleftDrive, backleftDrive, frontrightDrive, backrightDrive;
    private DcMotor intake, outLeft, outRight;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ===================== TUNING CONSTANTS =====================
    private static final double TARGET_RANGE = 10.0; // inches from robot center to tag

    private static final double Kp_DRIVE  = 0.05; // needs tuning
    private static final double Kp_STRAFE = 0.05; // needs tuning
    private static final double Kp_TURN   = 0.02; // needs tuning

    // ===================== CAMERA OFFSETS =====================
    // Measure these on your robot
    private static final double CAMERA_FORWARD_OFFSET = 6.0;  // inches (positive = camera in front)
    private static final double CAMERA_LEFT_OFFSET    = 0.0;  // inches (positive = camera left)

    @Override
    public void runOpMode() {

        // ===================== HARDWARE MAP =====================
        frontleftDrive  = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");
        backleftDrive   = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive  = hardwareMap.get(DcMotor.class, "br");

        outRight = hardwareMap.get(DcMotor.class, "or");
        outLeft  = hardwareMap.get(DcMotor.class, "ol");
        intake   = hardwareMap.get(DcMotor.class, "i");

        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();

        telemetry.addLine("Initialized – AprilTag Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double drive;
            double strafe;
            double turn;

            boolean autoAlign = gamepad1.right_bumper;

            List<AprilTagDetection> detections = aprilTag.getDetections();

            // ===================== AUTO ALIGN ====================
            if (autoAlign && !detections.isEmpty()) {

                AprilTagDetection tag = detections.get(0);

                // Correct camera-relative pose → robot center
                // ftcPose.range -> 3D distance from the camera lens to the AprilTag
                // tag.ftcPose.x -> left/right (sideways) offset of the AprilTag relative to the camera
                // ftcPose.bearing -> how much the robot needs to turn to face the tag

                double correctedRange = tag.ftcPose.range - CAMERA_FORWARD_OFFSET; // How far the camera lens is in front of (or behind) the robot’s center
                double correctedX     = tag.ftcPose.x + CAMERA_LEFT_OFFSET; // How far the camera lens is to the left (or right) of the robot’s center

                double rangeError   = correctedRange - TARGET_RANGE; //TARGET_RANGE --> the robot will try to stop so that its center is 10 inches away from the AprilTag
                // TARGET_RANGE = (robot_length / 2) + 2 inches
                double bearingError = tag.ftcPose.bearing; // Bearing -> degrees the robot need to turn to right/left to point the camera directly at the target

                drive  = clip(Kp_DRIVE  * rangeError);
                strafe = clip(Kp_STRAFE * correctedX);
                turn   = clip(Kp_TURN   * bearingError);

                telemetry.addLine("AUTO ALIGN ACTIVE");
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Range", "%.2f", correctedRange);
                telemetry.addData("Bearing", "%.2f", bearingError);
                telemetry.addData("X Offset", "%.2f", correctedX);

            }
            // ===================== MANUAL DRIVE =====================
            else {
                drive  = -gamepad1.left_stick_y;
                strafe =  gamepad1.left_stick_x;
                turn   =  gamepad1.right_stick_x;
            }

            // ===================== MECANUM DRIVE =====================
            double frpower = drive + turn - strafe;
            double brpower = drive + turn + strafe;
            double flpower = drive - turn + strafe;
            double blpower = drive - turn - strafe;

            double max = Math.max(
                    Math.abs(frpower),
                    Math.max(Math.abs(brpower),
                            Math.max(Math.abs(flpower), Math.abs(blpower)))
            );

            if (max > 1.0) {
                frpower /= max;
                brpower /= max;
                flpower /= max;
                blpower /= max;
            }

            frontrightDrive.setPower(frpower);
            backrightDrive.setPower(brpower);
            frontleftDrive.setPower(flpower);
            backleftDrive.setPower(blpower);

            // ===================== TELEMETRY =====================
            telemetry.addData("Mode", autoAlign ? "AUTO" : "MANUAL");
            telemetry.addData("Drive", "fl %.2f fr %.2f bl %.2f br %.2f",  // print only 2 digits after decimal
                    flpower, frpower, blpower, brpower);
            telemetry.update();
        }
    }

    // ===================== APRILTAG INIT =====================
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTag)
                .build();

        /*

        // Enable to preview AprilTag on driver hub.
        visionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
        .addProcessor(aprilTag)
        .setPreviewDisplayId(hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()))
        .build();

         */
    }

    // ===================== CLIP HELPER =====================
    private double clip(double value) {
        return Math.max(-1.0, Math.min(1.0, value)); // returns 1.0 if value grater than 1. returns -1.0 if value less than -1
    }
}
