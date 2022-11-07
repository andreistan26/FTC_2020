package test.GamepadTest;

import static java.lang.System.currentTimeMillis;

public class Button {

    public boolean value = false;
    public boolean lastValue = false;
    public boolean toggle = false;
    public int pressed = 0;
    public boolean intrat = false;
    public void updateValue(boolean value){
        lastValue = this.value;
        this.value = value;
        updateToggle();

    }

    public void updateToggle(){
        if (value == true && lastValue == false) {
            pressed = pressed + 1;
            if (toggle == false) {
                toggle = true;
            } else if (toggle == true) {
                toggle = false;
            }
        }

    }
}
