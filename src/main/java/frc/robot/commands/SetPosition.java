package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Position;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class SetPosition extends SequentialCommandGroup {

    public SetPosition(Wrist s_Wrist, Elevator s_Elevator, Position position, Supplier<GamePiece> gamePiece) {

        // Defaults to Stowed Position
        Command setElevatorPose = s_Elevator.setPose(Position.STANDBY.getElev());
        Command setWristPose = s_Wrist.setPose(Position.STANDBY.getWrist());
        boolean sequential = false;

        // Changes game piece if the position is an intaking position
        if(position == Position.STANDINGCONEINTAKE || position == Position.TIPPEDCONEINTAKE || 
           position == Position.HUMANPLAYERINTAKE || position == Position.CUBEINTAKE) {
            addCommands(new InstantCommand(() -> Intake.setGamePiece(gamePiece.get())));
        }

        switch (position) {
            case HIGH:
                if (gamePiece.get() == GamePiece.CONE) {
                    setWristPose = s_Wrist.setPose(Position.CONEHIGH.getWrist()); 
                    setElevatorPose = s_Elevator.setPose(Position.CONEHIGH.getElev());
                    sequential = true;                       
                } else if (gamePiece.get() == GamePiece.CUBE) {
                    setWristPose = s_Wrist.setPose(Position.CUBEHIGH.getWrist());
                    setElevatorPose = s_Elevator.setPose(Position.CUBEHIGH.getElev());
                }
                break;

            case MID:
                if (gamePiece.get() == GamePiece.CONE) {
                    setWristPose = s_Wrist.setPose(Position.CONEMID.getWrist());
                    setElevatorPose = s_Elevator.setPose(Position.CONEMID.getElev());
                } else if (gamePiece.get() == GamePiece.CUBE) {
                    setWristPose = s_Wrist.setPose(Position.CUBEMID.getWrist());
                    setElevatorPose = s_Elevator.setPose(Position.CUBEMID.getElev());
                }
                break;
            
            default:
                setWristPose = s_Wrist.setPose(position.getWrist());
                setElevatorPose = s_Elevator.setPose(position.getElev());
            
        }

        if(sequential) {
            addCommands(setElevatorPose, setWristPose);
        }
        else {
            addCommands(new ParallelCommandGroup(setElevatorPose, setWristPose));
        }
    }

}
