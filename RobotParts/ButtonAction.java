package RobotParts;

import test.GamepadTest.AVXGamepadTest;

import static RobotParts.FasterThanRR.AutoControl;

public class ButtonAction {
    AVXGamepadTest gamepad;
    Flicker flicker;
    Shooter shooter;

    public enum ButtonActionState{
        NOT_READY_TO_SHOOT, READY_TO_SHOOT
    }
    public ButtonActionState state;

    public ButtonAction(AVXGamepadTest gamepad, Flicker flicker, Shooter shooter) {
        this.gamepad = gamepad;
        this.flicker = flicker;
        this.shooter = shooter;
        state = ButtonActionState.NOT_READY_TO_SHOOT;
    }



    public void update() {

        if (shooter.shooterState == Shooter.ShooterState.OFF) {
            state = ButtonActionState.NOT_READY_TO_SHOOT;
        }
        if (shooter.shooterState == Shooter.ShooterState.AT_SPEED && flicker.stateTimeReady == Flicker.StateTimeReady.READY && flicker.statePosition== Flicker.StatePosition.IDLE) {
            state = ButtonActionState.READY_TO_SHOOT;
        }else{
            state = ButtonActionState.NOT_READY_TO_SHOOT;
        }
        powering();
    }

    public void powering(){
        if(gamepad.right_bumper.value){
            if(state == ButtonActionState.READY_TO_SHOOT) {
                flicker.flicker.setPosition(0.5);
            }
            if(shooter.shooterState == Shooter.ShooterState.OFF){
                shooter.shooterState = Shooter.ShooterState.GETTING_TO_SPEED;
            }

        }else if(!AutoControl){
            flicker.flicker.setPosition(0.58);
        }

        if (gamepad.left_bumper.value) {
            shooter.shooterState= Shooter.ShooterState.OFF;
        }

    }

    public boolean flick_auto(){
        if(state == ButtonActionState.READY_TO_SHOOT) {
            flicker.shoot();
                return true;
        }
        return false;
    }

}

