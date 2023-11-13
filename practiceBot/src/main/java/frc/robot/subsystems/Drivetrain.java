package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase{
    private CANSparkMax leftFront;
    private CANSparkMax leftRear;
    private CANSparkMax rightFront;
    private CANSparkMax rightRear;
    private DifferentialDrive diffDrive;
    private double speed;
    private double rotation;
    private boolean quickTurn;

    private RelativeEncoder leftFrontEncoder;
	private RelativeEncoder leftRearEncoder;
	private RelativeEncoder rightFrontEncoder;
	private RelativeEncoder rightRearEncoder;

    private Pigeon2 pigeon = new Pigeon2(0);
    private DifferentialDriveOdometry odometry;

    public Drivetrain(){
        initializeMotor(leftFront);
        initializeMotor(leftRear);
        initializeMotor(rightFront);
        initializeMotor(rightRear);

        setFollowers();

        leftFrontEncoder = this.leftFront.getEncoder();
		rightFrontEncoder = this.rightFront.getEncoder();
		leftRearEncoder = this.leftRear.getEncoder();
		rightRearEncoder = this.rightRear.getEncoder();

        diffDrive  = new DifferentialDrive(leftFront, rightFront);
        odometry = new DifferentialDriveOdometry(this.getRotation2d(), this.leftFrontEncoder.getPosition(), this.rightFrontEncoder.getPosition());
    }

    private void initializeMotor(CANSparkMax motor){
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
        motor.setOpenLoopRampRate(20);
    }
    
    private void setFollowers(){
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);
    }

    public void setCurvatureDrive(double speed, double rotation, boolean quickTurn){
        this.speed = speed;
        this.rotation = rotation;
        this.quickTurn = quickTurn;
    }

    public Rotation2d getRotation2d(){
        double degrees = -MathUtil.inputModulus(this.pigeon.getYaw(), -180, 180);
        return Rotation2d.fromDegrees(degrees);
    }

    public Pose2d getPose2d(){
        return this.odometry.getPoseMeters();
    }
    @Override
    public void periodic() {
        diffDrive.curvatureDrive(this.speed, this.rotation, this.quickTurn);
    }
}
