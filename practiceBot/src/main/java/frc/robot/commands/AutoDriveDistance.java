package frc.robot.commands;

import java.net.ProxySelector;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
        super.execute();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
