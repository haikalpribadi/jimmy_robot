package sjicontroller;      // <-- Required by the validation system

import sjicontrollerinterface.Customer;
import sjicontrollerinterface.JimmyInterface;

/*
 * @author Marc Davies
 */

public class Controller implements JimmyInterface {  // <-- Required by the validation system

    // Run if the customer selects the 'Accept' option for a delivered drink
    public void acceptDrink(String username, String deliveredDrinkName) {

        // This method doesn't require a response
        // Add code here to handle drinks that have been accepted

    }

    // Run if the customer selects the 'Change' option for a delivered drink
    public int changeDrink(String username, String requestedDrinkName) {

        return JimmyInterface.COMING_RIGHT_UP;

    }

    // Run when a new drink is ordered by the customer (after selection is made)
    public int getDrink(String username, String requestedDrinkName) {

        if (Customer.getTotalAccepted(username) < 2) {

            return JimmyInterface.SAY_NOTHING;

        }

        else if (Customer.getTotalDrinksUnconsumed(username, requestedDrinkName) == 2) {

            return JimmyInterface.YES_SIR;

        }

        else if (Customer.getTotalIncorrectClaims(username, requestedDrinkName) == 1) {

            return JimmyInterface.APOLOGY;

        }

        else if (Customer.getTotalRejected(username) < 4) {

            return JimmyInterface.QUESTION_THE_ORDER;

        }

        else if(Customer.getTotalDrinksOrdered(username) >= 10) {

            return JimmyInterface.REFUSE_THE_REQUEST;

        }

        else if (Customer.getTotalDrinksConsumed(username, requestedDrinkName) == 3) {

            return JimmyInterface.QUESTION_THE_ORDER;

        }

        else {

            return JimmyInterface.COMING_RIGHT_UP;

        }
    }

    // Run if the customer selects the 'Incorrect' option for a delivered drink
    public int incorrectDrink(String username, String deliveredDrinkName) {

        return JimmyInterface.APOLOGY;

    }

    // Run when a new drink is created by a replicator
    public boolean producedDrinkIsCorrect(String username, String producedDrinkName) {

        // Automatcally accepts every drink created by the replicator
        // Some drinks may not be correct but these are still accepted
        return true;

    }

    // Run if the customer selects the 'Reject' option for a delivered drink
    public int rejectDrink(String username, String deliveredDrinkName) {

        return JimmyInterface.SAY_NOTHING;

    }
}