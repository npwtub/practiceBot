package frc.robot.commands;

import java.net.ProxySelector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveDistance extends CommandBase{
    private Drivetrain drivetrain;
    private double posX;
    private ProfiledPIDController controller;

    public AutoDriveDistance(Drivetrain drivetrain, double posX){
        this.drivetrain = drivetrain;
        this.posX = posX;
        Constraints constraint = new Constraints(10, 10);
        controller = new ProfiledPIDController(0,0,0, constraint);
        controller.setTolerance(.2, 2);
        SmartDashboard.putData("Driving distance PID", controller);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller.reset(new State(0, 0));
        controller.setGoal(new State(posX, 0));
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        double rawOutput = this.controller.calculate(this.drivetrain.getPose2d().getX(), new State(posX ,0));
        rawOutput = MathUtil.clamp(rawOutput, -1, 1);
        this.drivetrain.setCurvatureDrive(rawOutput, 0, true);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if (controller.atGoal()) return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        this.drivetrain.setCurvatureDrive(0, 0, true);
    }
}