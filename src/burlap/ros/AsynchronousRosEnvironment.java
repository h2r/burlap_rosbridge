package burlap.ros;

import burlap.debugtools.DPrint;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.*;
import burlap.oomdp.core.objects.MutableObjectInstance;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.MutableState;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.common.NullRewardFunction;
import burlap.oomdp.singleagent.environment.Environment;
import burlap.oomdp.singleagent.environment.EnvironmentOutcome;
import com.fasterxml.jackson.databind.JsonNode;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/**
 * NOTE: This class is now deprecated! You should use the more general {@link burlap.ros.RosEnvironment}
 * or subclass the {@link burlap.ros.AbstractRosEnvironment} instead. If you want actions to
 * send their message as a string of their name, set the {@link burlap.ros.actionpub.ActionPublisher}
 * for all actions to a {@link burlap.ros.actionpub.ActionStringPublisher}.
 * <br/><br/>
 * An environment wrapper for state information provided over ROS with BURLAP actions
 * that are published to ROS. This environment connects to ROS using ROSBridge, which
 * must be running. State information
 * from ROS is expected to use be of
 * type burlap_msgs/burlap_state. The burlap_state message is parsed into an actual BURLAP
 * {@link burlap.oomdp.core.states.State} object using the object classes defined in a provided
 * BURLAP {@link burlap.oomdp.core.Domain}. This parsed state may be further modified before
 * the environment's current state is set to it by overriding the {@link #onStateReceive(burlap.oomdp.core.states.State)}
 * method, which receives the parsed state and returns a state object to which the environment's current
 * state will be updated.
 * <br/>
 * When this environment has an action request (via {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)}),
 * it turns the request into a {@link burlap.oomdp.singleagent.GroundedAction}
 * object and a string rep of the object is retrieved (via the {@link burlap.oomdp.singleagent.GroundedAction#toString()}
 * method, and then published to a ROS topic. The calling thread is then stalled for some delay (giving time
 * for the action to be executed on the ROS robot and the state updated) before the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)}
 * method returns. The fact that there is no "action completion" checking is why this is considered an asynchronous environment.
 * <br/>
 * After creating an environment, it may be a good idea to call the {@link #blockUntilStateReceived()} method
 * before doing anything with it. This method will block the calling thread until the environment receives a
 * state message from ROS which it uses to set its current state.
 * <br/>
 * A BURLAP reward function and terminal function can also be set
 * ({@link #setRewardFunction(burlap.oomdp.singleagent.RewardFunction)} and
 * {@link #setTerminalFunction(burlap.oomdp.core.TerminalFunction)}) so that the environment will returns meaningful messages
 * from the {@link #getLastReward()} and {@link #isInTerminalState()} methods.
 * <br/>
 * Note that the the environment's current state will be updated as frequently as ROSBridge provides
 * updates, but the states before and after actions returned are those from fixed time intervals set by the client
 * (and described above).
 * <br/>
 * You may want to specify a throttle rate and queue size for how quickly the environment receives state message
 * from ROS Bridge. If States are published too quickly, you may find the environment's state is lagging behind as
 * it processes each received state in order.
 *
 * @author James MacGlashan.
 */
@Deprecated
public class AsynchronousRosEnvironment implements Environment, RosListenDelegate{

	protected Domain			domain;
	protected RosBridge			rosBridge;
	protected Publisher			actionPub;
	protected int				actionSleepMS;

	protected State				curState;

	protected boolean			addHeader = false;

	protected RewardFunction	rf = new NullRewardFunction();
	protected TerminalFunction	tf = new NullTermination();

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
	 * @param rosActionTopic the name of the ROS topic to which BURLAP actions are published (as strings)
	 * @param actionSleepMS the amount of time that the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)} method stalls after publishing an action.
	 */
	public AsynchronousRosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic, String rosActionTopic, int actionSleepMS){

		this.domain = domain;


		this.rosBridge = RosBridge.createConnection(rosBridgeURI);
		this.rosBridge.waitForConnection();

		this.rosBridge.subscribe(rosStateTopic, "burlap_msgs/burlap_state", this);
		this.actionPub = new Publisher(rosActionTopic, "std_msgs/String", this.rosBridge);
		this.actionSleepMS = actionSleepMS;


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
	 * @param rosActionTopic the name of the ROS topic to which BURLAP actions are published (as strings)
	 * @param actionSleepMS the amount of time that the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)} method stalls after publishing an action.
	 * @param rosBridgeThrottleRate the ROS Bridge server throttle rate: how frequently the server will send state messages
	 * @param rosBridgeQueueLength the ROS Bridge queue length: how many messages are queued on the server; queueing is a consequence of the throttle rate
	 */
	public AsynchronousRosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic, String rosActionTopic,
									  int actionSleepMS, int rosBridgeThrottleRate, int rosBridgeQueueLength){

		this.domain = domain;


		this.rosBridge = RosBridge.createConnection(rosBridgeURI);
		this.rosBridge.waitForConnection();

		this.rosBridge.subscribe(rosStateTopic, "burlap_msgs/burlap_state", this, rosBridgeThrottleRate, rosBridgeQueueLength);
		this.actionPub = new Publisher(rosActionTopic, "std_msgs/String", this.rosBridge);
		this.actionSleepMS = actionSleepMS;


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
	 * @param rosActionTopic the name of the ROS topic to which BURLAP actions are published (as strings)
	 * @param rosStateTopicMessageType the ROS message type used for states.
	 * @param rosActionTopicMessageType the ROS message type used for actions.
	 * @param actionSleepMS the amount of time that the {@link #executeAction(burlap.oomdp.singleagent.GroundedAction)} method stalls after publishing an action.
	 * @param rosBridgeThrottleRate the ROS Bridge server throttle rate: how frequently the server will send state messages
	 * @param rosBridgeQueueLength the ROS Bridge queue length: how many messages are queued on the server; queueing is a consequence of the throttle rate
	 */
	public AsynchronousRosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic, String rosActionTopic,
									  String rosStateTopicMessageType, String rosActionTopicMessageType,
									  int actionSleepMS, int rosBridgeThrottleRate, int rosBridgeQueueLength){

		this.domain = domain;


		this.rosBridge = RosBridge.createConnection(rosBridgeURI);
		this.rosBridge.waitForConnection();

		this.rosBridge.subscribe(rosStateTopic, rosStateTopicMessageType, this, rosBridgeThrottleRate, rosBridgeQueueLength);
		this.actionPub = new Publisher(rosActionTopic, rosActionTopicMessageType, this.rosBridge);
		this.actionSleepMS = actionSleepMS;


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
	 * Specify whether a header object should be added to the action message. If yes, then the header will
	 * be retrieved from the method {@link #getHeader()} (which you can override). By default, the returned
	 * header is unpopulated, which will cause RosBridge to auto populate the sequence and time stamp (but leave
	 * the frame as an empty string).
	 * @param addHeader if true, then a header will be added to the action message; if false, no header is added.
	 */
	public void setAddHeader(boolean addHeader){
		this.addHeader = addHeader;
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

	@Deprecated
	public void receive(Map<String, Object> data, String stringRep) {

		//get the message content
		Map<String, Object> msgContent = (Map<String,Object>)data.get("msg");
		List<Map<String, Object>> burlapObjects = (List<Map<String, Object>>)msgContent.get("objects");

		State s = this.JSONPreparedToState(burlapObjects);
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


	@Override
	public EnvironmentOutcome executeAction(GroundedAction ga) {

		State startState = this.curState;

		String astr = ga.toString();

		final Map<String, Object> strData = new HashMap<String, Object>();
		if(this.addHeader){
			final Map<String, Object> header = this.getHeader();
			strData.put("header", header);
		}
		strData.put("data", astr);

		this.actionPub.publish(strData);

		try {
			Thread.sleep(this.actionSleepMS);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		State finalState = this.curState.copy();

		this.lastReward = this.rf.reward(startState, ga, finalState);

		EnvironmentOutcome eo = new EnvironmentOutcome(startState, ga, finalState, this.lastReward, this.tf.isTerminal(finalState));

		return eo;
	}


	@Override
	public double getLastReward() {
		return this.lastReward;
	}



	/**
	 * Specifies the header data that will be added to an action message if this environment has been set to
	 * add a header (by default headers are not added). Like all JSON messages, the header is represented as a map from the JSON field name
	 * to its values. A populated ROS header message map should have entries for the field names seq, time, and
	 * frame_id. By default, this method will return an empty Map, which will cause ROS Bridge to auto
	 * populate the seq and time fields and set the frame_id to the string "". If you want to set some
	 * of those values, override this method.
	 *
	 * @return a map specifying header field values.
	 */
	public Map<String, Object> getHeader(){
		return new HashMap<String, Object>();
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

	/**
	 * Takes a JSON prepared data structure from a ROS message representation of a state and turns it into a BURLAP state object. The JSON
	 * prepared version is a list of maps. Each map represents an object instance which stores the objects name ('name'), name
	 * of the object's class ('object_class'), a list of values ('values'). Each value is a map specifying the attribute name
	 * ('attribute') and its value in a string rep form ('value'). If the attribute is a MULTITARGETRELATIONAL type,
	 * then it is assumed the different object names to which it is pointing are separated by a single ',' without spaces.
	 * @param objects the list of OO-MDP object instances
	 * @return and OO-MDP {@link State} object.
	 */
	@Deprecated
	protected State JSONPreparedToState(List<Map<String, Object>> objects){

		State s = new MutableState();

		for(Map<String, Object> oMap : objects){
			String obName = (String)oMap.get("name");
			String className = (String)oMap.get("object_class");
			ObjectInstance ob = new MutableObjectInstance(this.domain.getObjectClass(className), obName);

			List<Map<String, Object>> values = (List<Map<String, Object>>)oMap.get("values");
			for(Map<String, Object> v : values){
				String aname = (String)v.get("attribute");
				String vv = (String)v.get("value");
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
