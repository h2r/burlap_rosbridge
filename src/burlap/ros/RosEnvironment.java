package burlap.ros;

import burlap.debugtools.DPrint;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.objects.MutableObjectInstance;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.MutableState;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.common.NullRewardFunction;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.EnvironmentOutcome;
import burlap.ros.actionpub.ActionPublisher;
import com.fasterxml.jackson.databind.JsonNode;
import ros.RosBridge;
import ros.RosListenDelegate;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/**
 * An {@link burlap.oomdp.singleagent.environment.Environment} in which state information is provided by ROS over ROS Bridge using
 * burlap_msgs/burlap_state message types. Actions are executed by specifying {@link burlap.ros.actionpub.ActionPublisher} instances
 * to handle the ROS communication for each given BURLAP {@link burlap.oomdp.singleagent.Action} (identified by the action's name).
 * The {@link burlap.ros.actionpub.ActionPublisher} objects are specified after this object is constructed using any of the following methods:
 * {@link #setActionPublisher(burlap.oomdp.singleagent.Action, burlap.ros.actionpub.ActionPublisher)},
 * {@link #setActionPublisher(String, burlap.ros.actionpub.ActionPublisher)}
 * {@link #setActionPublisherForMultipleAcitons(java.util.List, burlap.ros.actionpub.ActionPublisher)}, or
 * {@link #setActionPublisherForMultipleAcitonNames(java.util.List, burlap.ros.actionpub.ActionPublisher)}.
 * <br/><br/>
 * Before interacting with this environment, you should call the {@link #blockUntilStateReceived()} method to make sure it has received a state from ROS.
 * <br/><br/>
 * In the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)} method, first an associated {@link burlap.ros.actionpub.ActionPublisher} for the
 * provided {@link burlap.oomdp.singleagent.GroundedAction} is found. Then it calls the {@link burlap.ros.actionpub.ActionPublisher#publishAction(burlap.oomdp.singleagent.GroundedAction)}
 * method on it. When that method returns, the thread sleeps for the time delay returned by the method if the value is greater than 0. The resulting
 * action is set to whatever the latest burlap_msgs/burlap_state message received from ROS is and then the corresponding {@link burlap.oomdp.singleagent.environment.EnvironmentOutcome}
 * for the interaction is returned. The reward for the interaction is determined by a provided {@link burlap.oomdp.singleagent.RewardFunction} that by default is set to a
 * {@link burlap.oomdp.singleagent.common.NullRewardFunction} that always returns zero. The termination flag is set by querying this environment's
 * {@link #isInTerminalState()}, which queries a provided {@link burlap.oomdp.core.TerminalFunction} on the current environment state that by default is a
 * {@link burlap.oomdp.auxiliary.common.NullTermination} (always returns false).
 * <br/><br/>
 * Note that in the constructor you may want to set a low (e.g., 1) throttle rate and queue rate if burlap_msgs/burlap_state messages
 * are sent frequently, otherwise ROS Bridge may start lagging.
 * <br/><br/>
 * If you would like, you can override waiting for the first state message received from ROS and force this environment to report
 * a specific BURLAP {@link burlap.oomdp.core.states.State} using the {@link #overrideFirstReceivedState(burlap.oomdp.core.states.State)} method. Note
 * that any subsequent burlap_msgs/burlap_state messages will still update this environment's current state.
 * @author James MacGlashan.
 */
public class RosEnvironment implements Environment, RosListenDelegate{

	protected Domain domain;
	protected RosBridge rosBridge;
	protected Map<String, ActionPublisher> actionPublishers = new HashMap<String, ActionPublisher>();

	protected State curState;

	protected RewardFunction rf = new NullRewardFunction();
	protected TerminalFunction tf = new NullTermination();

	protected double			lastReward = 0.;

	protected Boolean			receivedFirstState = false;

	protected boolean			printStateAsReceived = false;

	protected int				debugCode = 7345252;




	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS. State information
	 * from ROS is expected to use be of
	 * type burlap_msgs/burlap_state. The burlap_state message is parsed into an actual BURLAP
	 * {@link burlap.oomdp.core.states.State} object using the object classes defined in a provided
	 * BURLAP {@link burlap.oomdp.core.Domain}.
	 * <br/>
	 * When this environment has an action request (via {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)}),
	 * it turns the request into a {@link burlap.oomdp.singleagent.GroundedAction}
	 * object and a string rep of the object is retrieved (via the {@link burlap.oomdp.singleagent.GroundedAction#toString()}
	 * method, and then published to a ROS topic. The calling thread is then stalled for some delay (giving time
	 * for the action to be executed on the ROS robot and the state updated) before the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)}
	 * method returns.
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param rosBridgeURI the URI of the ros bridge server. Note that by default, ros bridge uses port 9090. An example URI is ws://localhost:9090
	 * @param rosStateTopic the name of the ROS topic that publishes the burlap_msgs/burlap_state messages.
	 */
	public RosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic){

		this.domain = domain;


		this.rosBridge = RosBridge.createConnection(rosBridgeURI);
		this.rosBridge.waitForConnection();

		this.rosBridge.subscribe(rosStateTopic, "burlap_msgs/burlap_state", this);



	}

	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS. State information
	 * from ROS is expected to use be of
	 * type burlap_msgs/burlap_state. The burlap_state message is parsed into an actual BURLAP
	 * {@link burlap.oomdp.core.states.State} object using the object classes defined in a provided
	 * BURLAP {@link burlap.oomdp.core.Domain}.
	 * <br/>
	 * When this environment has an action request (via {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)}),
	 * it turns the request into a {@link burlap.oomdp.singleagent.GroundedAction}
	 * object and a string rep of the object is retrieved (via the {@link burlap.oomdp.singleagent.GroundedAction#toString()}
	 * method, and then published to a ROS topic. The calling thread is then stalled for some delay (giving time
	 * for the action to be executed on the ROS robot and the state updated) before the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)} )}
	 * method returns.
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param rosBridgeURI the URI of the ros bridge server. Note that by default, ros bridge uses port 9090. An example URI is ws://localhost:9090
	 * @param rosStateTopic the name of the ROS topic that publishes the burlap_msgs/burlap_state messages.
	 * @param rosBridgeThrottleRate the ROS Bridge server throttle rate: how frequently the server will send state messages
	 * @param rosBridgeQueueLength the ROS Bridge queue length: how many messages are queued on the server; queueing is a consequence of the throttle rate
	 */
	public RosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic, int rosBridgeThrottleRate, int rosBridgeQueueLength){

		this.domain = domain;


		this.rosBridge = RosBridge.createConnection(rosBridgeURI);
		this.rosBridge.waitForConnection();

		this.rosBridge.subscribe(rosStateTopic, "burlap_msgs/burlap_state", this, rosBridgeThrottleRate, rosBridgeQueueLength);

	}

	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS. Note that although you can specify a different message
	 * type for the state and action using this constructor, the State message should adhere to the same
	 * type in as the burlap_msgs/burlap_state (State information
	 * from ROS is expected to use be of
	 * type burlap_msgs/burlap_state (https://github.com/h2r/burlap_msgs) and the action topic message type
	 * should either be a std/String, or a message adhering to the time stamped string burlap_msgs/timed_action
	 * (https://github.com/h2r/burlap_msgs).
	 * The burlap_state message is parsed into an actual BURLAP
	 * {@link burlap.oomdp.core.states.State} object using the object classes defined in a provided
	 * BURLAP {@link burlap.oomdp.core.Domain}.
	 * <br/>
	 * When this environment has an action request (via {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)}),
	 * it turns the request into a {@link burlap.oomdp.singleagent.GroundedAction}
	 * object and a string rep of the object is retrieved (via the {@link burlap.oomdp.singleagent.GroundedAction#toString()}
	 * method, and then published to a ROS topic. The calling thread is then stalled for some delay (giving time
	 * for the action to be executed on the ROS robot and the state updated) before the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)}
	 * method returns.
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param rosBridgeURI the URI of the ros bridge server. Note that by default, ros bridge uses port 9090. An example URI is ws://localhost:9090
	 * @param rosStateTopic the name of the ROS topic that publishes the burlap_msgs/burlap_state messages.
	 * @param rosStateTopicMessageType the ROS message type used for states.
	 * @param rosBridgeThrottleRate the ROS Bridge server throttle rate: how frequently the server will send state messages
	 * @param rosBridgeQueueLength the ROS Bridge queue length: how many messages are queued on the server; queueing is a consequence of the throttle rate
	 */
	public RosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic,
									  String rosStateTopicMessageType, int rosBridgeThrottleRate, int rosBridgeQueueLength){

		this.domain = domain;


		this.rosBridge = RosBridge.createConnection(rosBridgeURI);
		this.rosBridge.waitForConnection();

		this.rosBridge.subscribe(rosStateTopic, rosStateTopicMessageType, this, rosBridgeThrottleRate, rosBridgeQueueLength);

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


	/**
	 * Sets a BURLAP {@link burlap.oomdp.singleagent.RewardFunction} to use to provide reward signals from the {@link #getLastReward()} method.
	 * @param rf {@link burlap.oomdp.singleagent.RewardFunction} to use to provide reward signals.
	 */
	public void setRewardFunction(RewardFunction rf){
		this.rf = rf;
	}

	/**
	 * Sets the BURLAP {@link burlap.oomdp.core.TerminalFunction} to use to provide terminal state checks from the {@link #isInTerminalState()}.
	 * @param tf the BURLAP {@link burlap.oomdp.core.TerminalFunction} to use to provide terminal state checks
	 */
	public void setTerminalFunction(TerminalFunction tf){
		this.tf = tf;
	}

	@Override
	public State getCurrentObservation() {
		return this.curState.copy();
	}


	@Override
	public boolean isInTerminalState() {
		return this.tf.isTerminal(this.curState);
	}

	@Override
	public void resetEnvironment() {
		//do nothing...
	}


	@Override
	public EnvironmentOutcome executeAction(GroundedAction ga) {

		State startState = this.curState;

		ActionPublisher ap = this.actionPublishers.get(ga.actionName());
		if(ap == null){
			throw new RuntimeException("ROSEnvironment has no ActionPublisher available to handle action " + ga.toString());
		}

		int delay = ap.publishAction(ga);
		if(delay > 0){
			try {
				Thread.sleep(delay);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		State finalState = this.curState.copy();

		this.lastReward = this.rf.reward(startState, ga, finalState);

		EnvironmentOutcome eo = new EnvironmentOutcome(startState, ga, finalState, this.lastReward, this.isInTerminalState());

		return eo;
	}

	@Override
	public double getLastReward() {
		return this.lastReward;
	}

	/**
	 * Sets whether the default implementation of {@link #onStateReceive(burlap.oomdp.core.states.State)} will print the
	 * state information to the terminal.
	 * @param printStateAsReceived if true, then the default implementation of {@link #onStateReceive(burlap.oomdp.core.states.State)} will print states to the screen;
	 *                             if false, the state receiving is silent by default.
	 */
	public void setPrintStateAsReceived(boolean printStateAsReceived){
		this.printStateAsReceived = printStateAsReceived;
	}


	/**
	 * Use this method to override the first received state from ROS to be the provided {@link burlap.oomdp.core.states.State}.
	 * If the {@link #blockUntilStateReceived()} method has been called in another thread, it will be notified that the first state is received.
	 * Note that if ROS Bridge subsequently sends state message, the state of this environment will be updated to reflect them.
	 * @param s the {@link burlap.oomdp.core.states.State} to force this Environment to.
	 */
	public void overrideFirstReceivedState(State s){
		this.curState = s;
		this.receivedFirstState = true;
		synchronized (this){
			this.notifyAll();
		}
	}

	/**
	 * A method you can call that forces the calling thread to wait until the first state from ROS has been received.
	 */
	public synchronized void blockUntilStateReceived(){
		DPrint.cl(this.debugCode, "Blocking until state received.");
		while(!this.receivedFirstState){
			try {
				this.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		DPrint.cl(this.debugCode, "State received");
	}


	@Override
	public void receive(JsonNode data, String stringRep) {
		JsonNode burlapObjects = data.get("msg").get("objects");
		State s = this.JSONToState(burlapObjects);
		this.curState = this.onStateReceive(s);

		if(!this.receivedFirstState){
			synchronized (this){
				this.receivedFirstState = true;
				this.notifyAll();
			}
		}

	}


	/**
	 * This method is called whenever a state message from ROS is received. The new current state of the environment
	 * will be set to whatever this method returns. By default, this method simply returns the same reference and
	 * if this environment's {@link #printStateAsReceived} data member is set to true, then it will print
	 * to the terminal the string representation of the state. Override this method to provide
	 * special handling of used state (e.g., adding objects to the state that ROS does not perceive).
	 * @param s the parsed state from the ROS message received.
	 */
	protected State onStateReceive(State s){
		if(printStateAsReceived) {
			System.out.println(s.getCompleteStateDescription() + "\n-------------------------");
		}
		return s;
	}

	/**
	 * Takes a {@link com.fasterxml.jackson.databind.JsonNode} that represents the BURLAP state and turns it into
	 * a BURLAP {@link burlap.oomdp.core.states.State} object. The JSonNode at the top level is a list of objects.
	 * Each object has a map with the fields "name", "object_class", and "values". The former are just string definitions.
	 * The latter is another list of values. Each value has the fields "attribute" and "value". Attribute specified the name
	 * of the BURLAP attribute; "value" specifies the string value of the value. If the attribute is a MULTITARGETRELATIONAL type,
	 * then it is assumed the different object names to which it is pointing are separated by a single ',' without spaces.
	 * @param objects a {@link com.fasterxml.jackson.databind.JsonNode} specifying the BURLAP state objects.
	 * @return A BURLAP {@link burlap.oomdp.core.states.State} representation of the input JSON state.
	 */
	protected State JSONToState(JsonNode objects){
		State s = new MutableState();

		Iterator<JsonNode> objIter = objects.elements();
		while(objIter.hasNext()){
			JsonNode obj = objIter.next();
			String obName = obj.get("name").asText();
			String className = obj.get("object_class").asText();
			ObjectInstance ob = new MutableObjectInstance(this.domain.getObjectClass(className), obName);

			JsonNode values = obj.get("values");
			Iterator<JsonNode> valueIter = values.elements();
			while(valueIter.hasNext()){
				JsonNode v = valueIter.next();
				String aname = v.get("attribute").asText();
				String vv = v.get("value").asText();
				if(this.domain.getAttribute(aname).type != Attribute.AttributeType.MULTITARGETRELATIONAL) {
					ob.setValue(aname, vv);
				}
				else{
					String [] targets = vv.split(",");
					for(String t : targets){
						ob.addRelationalTarget(aname, t);
					}
				}
			}
			s.addObject(ob);

		}

		return s;

	}


}
