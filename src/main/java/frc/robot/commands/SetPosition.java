package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Position;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SetPosition extends CommandBase {
    Wrist s_Wrist;
    Elevator s_Elevator;
    Position position;
    Supplier<GamePiece> gamePiece;

    public SetPosition(Wrist s_Wrist, Elevator s_Elevator, Position position, Supplier<GamePiece> gamePiece) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        this.position = position;
        this.gamePiece = gamePiece;
        }

    @Override
    public void execute() {
        switch (position) {
            case HIGH:
                if (gamePiece.get() == GamePiece.CONE) {
                    s_Wrist.setPosition(.104327);
                    s_Elevator.setPosition(35.1);
                } else if (gamePiece.get() == GamePiece.CUBE) {
                    s_Wrist.setPosition(1.55);
                    s_Elevator.setPosition(35.1);
                }
                break;

            case MID:
                if (gamePiece.get() == GamePiece.CONE) {
                    s_Wrist.setPosition(.104327);
                    s_Elevator.setPosition(23);
                } else if (gamePiece.get() == GamePiece.CUBE) {
                    s_Wrist.setPosition(1.427);
                    s_Elevator.setPosition(16.5);
                }
                break;

            case LOW:
                s_Wrist.setPosition(0.5236);
                s_Elevator.setPosition(0);
                break;

            case STANDBY:
                s_Wrist.setPosition(1.1765);
                s_Elevator.setPosition(0);
                break;

            case CUBEINTAKE:
                s_Wrist.setPosition(0);
                s_Elevator.setPosition(0);
                break;

            case STANDINGCONEINTAKE:
                s_Wrist.setPosition(5.106);
                s_Elevator.setPosition(14.380);
                break;

            case TIPPEDCONEINTAKE:
                s_Wrist.setPosition(5.572);
                s_Elevator.setPosition(1.333);
                break;

            case HUMANPLAYERINTAKE:
                s_Wrist.setPosition(.8763);
                s_Elevator.setPosition(3.5472);
                break;
        }

    }

    @Override
    public boolean isFinished() {
        return s_Wrist.atSetpoint() && s_Elevator.atSetpoint();
    }
}
