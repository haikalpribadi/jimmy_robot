package sjicontrollerinterface;

/*
 * @author Marc Davies
 */

public abstract interface JimmyInterface {

    public static final int COMING_RIGHT_UP = 0;
    public static final int APOLOGY = 1;
    public static final int YES_SIR = 2;
    public static final int SAY_NOTHING = 3;
    public static final int QUESTION_THE_ORDER = 4;
    public static final int REFUSE_THE_REQUEST = 5;

    public abstract void acceptDrink(String paramString1, String paramString2);

    public abstract int changeDrink(String paramString1, String paramString2);

    public abstract int getDrink(String paramString1, String paramString2);

    public abstract int incorrectDrink(String paramString1, String paramString2);

    public abstract boolean producedDrinkIsCorrect(String paramString1, String paramString2);

    public abstract int rejectDrink(String paramString1, String paramString2);
}
