package overcharged.components;

/**
 * General Button definitions
 */
public class Button
{
    public final static Button CLAW = new Button();
    public final static Button INTAKETILT = new Button();


    public static final int BTN_PRESS_INTERVAL = 500; // milliseconds
    public static final float TRIGGER_THRESHOLD = (float)0.75;
    public static final Button BTN_DISABLE = new Button();
    public static final Button BTN_RIGHT = new Button();
    public static final Button BTN_LEFT = new Button();

    public long lastPressTime;

    boolean lastButton = false;
    boolean isToggled = false;

    /**
     * standard press function.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @return if the button is pressable yet
     */
    public boolean canPress(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL);
    }

    /**
     * press function for half time.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @return if the button is pressable yet
     */
    public boolean canPressShort(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL/2);
    }

    /**
     * press function for fourth time.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @return if the button is pressable yet
     */
    public boolean canPress4Short(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL/4);
    }

    /**
     * press function for eighth time.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @return if the button is pressable yet
     */
    public boolean canPress6Short(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL/6);
    }

    /**
     * determine if the button has become pressable.
     * Also update the internal timestamp.
     * @param timeStamp timestamp to use
     * @param interval interval from previous button press
     * @return if the button is pressable yet
     */
    public boolean canPress(long timeStamp,
                            long interval) {
        boolean pressable = isPressable(timeStamp, interval);
        if(pressable) {
            lastPressTime = timeStamp;
        }
        return pressable;
    }

    /**
     * determine if the button has become pressable
     * without updating internal timestamp
     * @param timeStamp
     * @param interval
     * @return
     */
    public boolean isPressable(long timeStamp,
                               long interval) {
        return (timeStamp - lastPressTime) > interval;
    }

    public boolean isPressed(boolean button) {
        boolean a = button && !lastButton;
        lastButton = button;
        return a;
    }

    public boolean isToggled(boolean button) {
        if (isPressed(button)) {
            isToggled = !isToggled;
        }
        return isToggled;
    }

    public boolean isReleased(boolean button) {
        boolean a = !button && lastButton;
        lastButton = button;
        return a;
    }
}
