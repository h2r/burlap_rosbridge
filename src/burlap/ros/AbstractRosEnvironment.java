package burlap.ros;

import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.EnvironmentOutcome;
import burlap.ros.actionpub.ActionPublisher;
import ros.RosBridge;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * An abstract {@link burlap.oomdp.singleagent.environment.Environment} implementation that executes
 * actions on robots controlled by ROS via ROS Bridge. Note that this abstract class does not implement
 * the {@link burlap.oomdp.singleagent.environment.Environment} methods
 * {@link burlap.oomdp.singleagent.environment.Environment#getCurrentObservation()} or
 * {@link burlap.oomdp.singleagent.environment.Environment#isInTerminalState()} and
 * the {@link burlap.oomdp.singleagent.environment.Environment#getLastReward()} method requires this class's
 * abstract method {@link #getMostRecentRewardSignal(burlap.oomdp.core.states.State, burlap.oomdp.singleagent.GroundedAction, burlap.oomdp.core.states.State)} to be implemented.
 * Instead, the primary purpose of this class is to provide a general framework for handling action execution on ROS via the {@link burlap.ros.actionpub.ActionPublisher}
 * methodology. Specifically, for each BURLAP action (identified by its name), an {@link burlap.ros.actionpub.ActionPublisher} may be specified using one of the following
 * methods:<br/>
 * {@link #setActionPublisher(burlap.oomdp.singleagent.Action, burlap.ros.actionpub.ActionPublisher)},<br/>
 * {@link #setActionPublisher(String, burlap.ros.actionpub.ActionPublisher)}<br/>
 * {@link #setActionPublisherForMultipleAcitons(java.util.List, burlap.ros.actionpub.ActionPublisher)}, or<br/>
 * {@link #setActionPublisherForMultipleAcitonNames(java.util.List, burlap.ros.actionpub.ActionPublisher)}.<br/>
 * When {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)} is called, the corresponding {@link burlap.ros.actionpub.ActionPublisher}
 * for the provided {@link burlap.oomdp.singleagent.GroundedAction} is retrieved (with a runtime exception being thrown if none have been set for it).
 * The {@link burlap.ros.actionpub.ActionPublisher#publishAction(burlap.oomdp.singleagent.GroundedAction)} is then called and passed the input
 * {@link burlap.oomdp.singleagent.GroundedAction}. After the method returns, the thread sleeps (and blocks) for the amount of time in
 * milliseconds specified by the return value of the method so long as it is greater than zero. The returned {@link burlap.oomdp.singleagent.environment.EnvironmentOutcome}
 * object is formed by querying the {@link #getCurrentObservation()} method before the {@link burlap.ros.actionpub.ActionPublisher} is invoked for the pre-state
 * and after the thread sleeping completes for the post-state; this object's abstract {@link #getMostRecentRewardSignal(burlap.oomdp.core.states.State, burlap.oomdp.singleagent.GroundedAction, burlap.oomdp.core.states.State)}
 * method is then called to get the reward signal to use (and is provided the state-action-state transition that was just recorded, which may or may not be needed); and the
 * the terminal state flag is set by invoking this object's {@link #isInTerminalState()}. If the environment transitions to a terminal state,
 * then before the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)} method exits, it calls the
 * {@link #handleEnterTerminalState()} abstract method. This latter method does not have to be implemented, but allows you
 * to easily inject code for handling the special case when the robot enters a terminal state. Therefore, as long as all necessary methods for this abstract class
 * are implemented, everything will work.
 *
 * @author James MacGlashan.
 */
public abstract class AbstractRosEnvironment implements Environment{

	/**
	 * The {@link ros.RosBridge} connection
	 */
	protected RosBridge rosBridge;

	/**
	 * The mapping from BURLAP {@link burlap.oomdp.singleagent.Action} names to the corresponding
	 * {@link burlap.ros.actionpub.ActionPublisher} to use.
	 */
	protected Map<String, ActionPublisher> actionPublishers = new HashMap<String, ActionPublisher>();

	/**
	 * The last reward signal generated.
	 */
	protected double lastReward = 0.;

	/**
	 * Initializes with the given {@link ros.RosBridge}.
	 * @param rosBridge the {@link ros.RosBridge} connection to use.
	 */
	public AbstractRosEnvironment(RosBridge rosBridge) {
		this.rosBridge = rosBridge;
	}

	/**
	 * Creates a {@link ros.RosBridge} connection for this {@link burlap.oomdp.singleagent.environment.Environment} at the given ROS Bridge URI.
	 * and blocks until a connection with has been established.
	 * @param rosBridgeURI the ROS Bridge URI; e.g., ws://localhost:9090
	 */
	public AbstractRosEnvironment(String rosBridgeURI) {
		this.rosBridge = RosBridge.createConnection(rosBridgeURI);
		this.rosBridge.waitForConnection();
	}


	/**
	 * Sets the {@link burlap.ros.actionpub.ActionPublisher} to handle executions of actions with the name actionName
	 * @param actionName the name of the action
	 * @param ap the {@link burlap.ros.actionpub.ActionPublisher} that handles executions of actions with that name
	 */
	public void setActionPublisher(String actionName, ActionPublisher ap){
		this.actionPublishers.put(actionName, ap);
	}

	/**
	 * Sets the {@link burlap.ros.actionpub.ActionPublisher} to handle executions of the given {@link burlap.oomdp.singleagent.Action}
	 * @param action the {@link burlap.oomdp.singleagent.Action} to handle
	 * @param ap the {@link burlap.ros.actionpub.ActionPublisher} to handle the executions
	 */
	public void setActionPublisher(Action action, ActionPublisher ap){
		this.actionPublishers.put(action.getName(), ap);
	}


	/**
	 * Sets a single {@link burlap.ros.actionpub.ActionPublisher} to handle the execution of a list of {@link burlap.oomdp.singleagent.Action} objects.
	 * @param actions the {@link burlap.oomdp.singleagent.Action} objects to handle
	 * @param ap the {@link burlap.ros.actionpub.ActionPublisher} that handles the execution
	 */
	public void setActionPublisherForMultipleAcitons(List<Action> actions, ActionPublisher ap){
		for(Action a : actions){
			this.setActionPublisher(a, ap);
		}
	}


	/**
	 * Sets a single {@link burlap.ros.actionpub.ActionPublisher} to handle the execution of a list of actions identified by given action names.
	 * @param actionNames the list of action names to handle
	 * @param ap the {@link burlap.ros.actionpub.ActionPublisher} that handles execution
	 */
	public void setActionPublisherForMultipleAcitonNames(List<String> actionNames, ActionPublisher ap){
		for(String a : actionNames){
			this.setActionPublisher(a, ap);
		}
	}

	/**
	 * Returns the {@link ros.RosBridge} object to which this environment is connected.
	 * @return the {@link ros.RosBridge} object to which this environment is connected.
	 */
	public RosBridge getRosBridge(){
		return this.rosBridge;
	}


	@Override
	public EnvironmentOutcome executeAction(GroundedAction ga) {

		State startState = this.getCurrentObservation();

		ActionPublisher ap = this.actionPublishers.get(ga.actionName());
		if(ap == null){
			throw new RuntimeException("AbstractRosEnvironment has no ActionPublisher available to handle action " + ga.toString());
		}

		int delay = ap.publishAction(ga);
		if(delay > 0){
			try {
				Thread.sleep(delay);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		State finalState = this.getCurrentObservation();

		this.lastReward = this.getMostRecentRewardSignal(startState, ga, finalState);

		EnvironmentOutcome eo = new EnvironmentOutcome(startState, ga, finalState, this.lastReward, this.isInTerminalState());

		if(this.isInTerminalState()){
			this.handleEnterTerminalState();
		}

		return eo;
	}

	@Override
	public double getLastReward() {
		return this.lastReward;
	}

	@Override
	public void resetEnvironment() {
		this.lastReward = 0.;
	}

	/**
	 * Generates the reward signal for the most recent environment interaction initiated through the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)}
	 * method and what will
	 * be returned by the {@link #getLastReward()} method until another {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)} method
	 * is invoked or this environment is reset with {@link #resetEnvironment()}.
	 * @param s the previous environment state before this reward event
	 * @param ga the action taken by the agent before this reward event
	 * @param sprime the resulting environment state associated with this reward event
	 * @return the reward for the most recent environment interaction
	 */
	protected abstract double getMostRecentRewardSignal(State s, GroundedAction ga, State sprime);


	/**
	 * This method is called just before exiting the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)} method
	 * if the newly entered environment state is a terminal state. This method does not have to do anything, but is useful
	 * if you need to do something special with the robot when it reaches a terminal state.
	 */
	protected abstract void handleEnterTerminalState();


}
