package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.Subsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class drivetrain extends Subsystem{
    private CommandXboxController driver =new CommandXboxController(0);

    private final SwerveRequest.FieldCentricFacingAngle facingAngle =
        new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public drivetrain() {
        facingAngle.HeadingController.setPID(7.0, 0.0, 0.0);
            facingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
}
    private static Translation2d robotPos = LimelightHelpers.getBotPose2d("limelight").getTranslation();
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private static CommandSwerveDrivetrain instance;
    public static CommandSwerveDrivetrain getInstance(){
        if(instance==null){
            instance=TunerConstants.createDrivetrain();
        }
        return instance;
    }
    public static CommandSwerveDrivetrain subsystem(){
        return getInstance();
    }
    private state currentState = state.idle;

    public enum state {
        idle(0),
        moving(1),
        shootingOnTheMove(2);

        public final int stateNum;

        state(int stateNum) {
            this.stateNum = stateNum;
        }
    }

    public Runnable[][] methods = {
            {
                    () -> emptyMethod(),
                    () -> emptyMethod(),
                    () -> emptyMethod()
            },

            {
                    () -> emptyMethod(),
                    () -> emptyMethod(),
                    () -> emptyMethod()
            },
            {
                    () -> turnToAngle(()-> -driver.getLeftY() * MaxSpeed,()->-driver.getLeftX() * MaxSpeed ),
                    () -> turnToAngle(()-> -driver.getLeftY() * MaxSpeed,()->-driver.getLeftX() * MaxSpeed ),
                    () -> emptyMethod()
            }
    };

    public void changeState(state newState) {
        if (this.currentState == newState)
            return;
        methods[currentState.stateNum][2].run();
        currentState = newState;
        methods[currentState.stateNum][0].run();
        super.activeStatePeriodic = methods[newState.stateNum][1];
    }
    private static Rotation2d setPos(){
        ChassisSpeeds speed = CommandSwerveDrivetrain.getSwerve().getState().Speeds;
        Translation2d hubx= new Translation2d(4.0+speed.vxMetersPerSecond,18.0+speed.vyMetersPerSecond);
        Rotation2d degree = hubx.minus(robotPos).getAngle();
        return degree;
    }
    public Command turnToAngle(DoubleSupplier xHiz, DoubleSupplier yHiz) {
    return drivetrain.getInstance().applyRequest(() ->
                    facingAngle
                            .withVelocityX(xHiz.getAsDouble())
                            .withVelocityY(yHiz.getAsDouble())
                            .withTargetDirection(setPos()))
            .until(() -> Math.abs(
                    setPos().minus(drivetrain.getInstance().getState().Pose.getRotation()).getDegrees()) < 2.0);
}

}
