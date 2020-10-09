package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

public class Concensus {
    private Map<String, Boolean> votes;
    private ConcensusMode mode;
    /**
     * Defines how the general concensus is actually calculated
     */
    public enum ConcensusMode {
        /**
         * Concensus is true if the majority of 'votes' is true
         */
        Majority,

        /**
         * <p>Concensus is true if all the 'votes' are true.</p>
         * <p>Likewise, if any class 'votes' false, Concensus is false</p>
         */
        All,

        /**
         * <p>Concensus is true if any of the 'votes' are true</p>
         * <p>Likewise, if all classes 'vote' false, Concensus is false</p>
         */
        Any
    }
    /**
     * create a new Concensus object with the specified mode
     */
    public Concensus(ConcensusMode mode) {
        votes = new HashMap<String, Boolean>();
        this.mode = mode;
    }
    /**
     * This casts a 'vote' from the class this method is called in.
     * The effect of 'votes' is defined by the ConcensusMode
     */
    public void vote(boolean shouldRun) {
        votes.put(new Exception().getStackTrace()[2].getClassName(), shouldRun);
    }
    /**
     * <br>returns the global concensus on the issue</br>
     * <ul>
     * <li>All: true if all the 'votes' are true</li>
     * <li>Any: true if any of the'votes' are true</li>
     * <li>Majority: true if the majority of 'votes' is true</li>
     * </ul>
     */
    public boolean getConcensus() {
        switch(mode) {
            case All:
                for(Boolean b : votes.values()) {
                    if(!b) return false;
                }
                return true;
            case Any:
                for(Boolean b : votes.values()) {
                    if(b) return true;
                }
                return false;
            case Majority:
                int count = 0;
                for(Boolean b : votes.values()) {
                    if(b) count++;
                }
                return count > votes.size() / 2;

        }
        return false;
    }
}