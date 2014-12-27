package burlap.ros;

import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.common.NullRewardFunction;
import burlap.oomdp.singleagent.environment.Environment;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 *
 * An environment wrapper for state information provided over ROS with BURLAP actions
 * that are published to ROS. This environment connects to ROS using ROSBridge, which
 * must be running. State information
 * from ROS is expected to use be of
 * type burlap_msgs/burlap_state. The burlap_state message is parsed into an actual BURLAP
 * {@link burlap.oomdp.core.State} object using the object classes defined in a provided
 * BURLAP {@link burlap.oomdp.core.Domain}.
 * <br/>
 * When this environment has an action request (via {@link #executeAction(String, String[])}),
 * it turns the request into a {@link burlap.oomdp.singleagent.GroundedAction}
 * object and a string rep of the object is retrieved (via the {@link burlap.oomdp.singleagent.GroundedAction#toString()}
 * method, and then published to a ROS topic. The calling thread is then stalled for some delay (giving time
 * for the action to be executed on the ROS robot and the state updated) before the {@link #executeAction(String, String[])}
 * method returns.
 * <br/>
 * After creating an environment, it may be a good idea to call the {@link #blockUntilStateReceived()} method
 * before doing anything with it. This method will block the calling thread until the environment receives a
 * state message from ROS which it uses to set its current state.
 * <br/>
 * A BURLAP reward function and terminal function can also be set
 * ({@link #setRewardFunction(burlap.oomdp.singleagent.RewardFunction)} and
 * {@link #setTerminalFunction(burlap.oomdp.core.TerminalFunction)}) so that the environment will returns meaningful messages
 * from the {@link #getLastReward()} and {@link #curStateIsTerminal()} methods.
 * <br/>
 * Note that the the environment's current state will be updated as frequently as ROSBridge provides
 * updates, but the states before and after actions returned are those from fixed time intervals set by the client
 * (and described above).
 *
 * @author James MacGlashan.
 */
public class RosEnvironment extends Environment implements RosListenDelegate{

	protected Domain			domain;
	protected RosBridge			rosBridge;
	protected Publisher			actionPub;
	protected int				actionSleepMS;

	protected RewardFunction	rf = new NullRewardFunction();
	protected TerminalFunction	tf = new NullTermination();

	protected double			lastReward = 0.;

	protected Boolean			receivedFirstState = false;


	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS. State information
	 * from ROS is expected to use be of
	 * type burlap_msgs/burlap_state. The burlap_state message is parsed into an actual BURLAP
	 * {@link burlap.oomdp.core.State} object using the object classes defined in a provided
	 * BURLAP {@link burlap.oomdp.core.Domain}.
	 * <br/>
	 * When this environment has an action request (via {@link #executeAction(String, String[])}),
	 * it turns the request into a {@link burlap.oomdp.singleagent.GroundedAction}
	 * object and a string rep of the object is retrieved (via the {@link burlap.oomdp.singleagent.GroundedAction#toString()}
	 * method, and then published to a ROS topic. The calling thread is then stalled for some delay (giving time
	 * for the action to be executed on the ROS robot and the state updated) before the {@link #executeAction(String, String[])}
	 * method returns.
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param rosBridgeURI the URI of the ros bridge server. Note that by default, ros bridge uses port 9090. An example URI is ws://localhost:9090
	 * @param rosStateTopic the name of the ROS topic that publishes the burlap_msgs/burlap_state messages.
	 * @param rosActionTopic the name of the ROS topic to which BURLAP actions are published (as strings)
	 * @param actionSleepMS the amount of time that the {@link #executeAction(String, String[])} method stalls after publishing an action.
	 */
	public RosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic, String rosActionTopic, int actionSleepMS){

		this.domain = domain;


		this.rosBridge = RosBridge.createConnection(rosBridgeURI);
		this.rosBridge.waitForConnection();

		this.rosBridge.subscribe(rosStateTopic, "burlap_msgs/burlap_state", this);
		this.actionPub = new Publisher(rosActionTopic, "std_msgs/String", this.rosBridge);
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
	 * Sets the BURLAP {@link burlap.oomdp.core.TerminalFunction} to use to provide terminal state checks from the {@link #curStateIsTerminal()}.
	 * @param tf the BURLAP {@link burlap.oomdp.core.TerminalFunction} to use to provide terminal state checks
	 */
	public void setTerminalFunction(TerminalFunction tf){
		this.tf = tf;
	}

	/**
	 * A method you can call that forces the calling thread to wait until the first state from ROS has been received.
	 */
	public synchronized void blockUntilStateReceived(){
		while(!this.receivedFirstState){
			try {
				this.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	@Override
	public void receive(Map<String, Object> data, String stringRep) {

		//get the message content
		Map<String, Object> msgContent = (Map<String,Object>)data.get("msg");
		List<Map<String, Object>> burlapObjects = (List<Map<String, Object>>)msgContent.get("objects");
		this.curState = this.JSONPreparedToState(burlapObjects);


		if(!this.receivedFirstState){
			synchronized (this){
				this.receivedFirstState = true;
				this.notifyAll();
			}
		}

	}

	@Override
	public State executeAction(String aname, String[] params) {

		State startState = this.curState;

		GroundedAction ga = new GroundedAction(this.domain.getAction(aname), params);
		String astr = ga.toString();

		final Map<String, String> strData = new HashMap<String, String>();
		strData.put("data", astr);
		this.actionPub.publish(strData);

		try {
			Thread.sleep(this.actionSleepMS);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		State finalState = this.curState.copy();

		this.lastReward = this.rf.reward(startState, ga, finalState);

		return finalState;
	}

	@Override
	public double getLastReward() {
		return this.lastReward;
	}

	@Override
	public boolean curStateIsTerminal() {
		return this.tf.isTerminal(this.curState);
	}



	/**
	 * Takes a JSON prepared data structure from a ROS message representation of a state and turns it into a BURLAP state object. The JSON
	 * prepared version is a list of maps. Each map represents an object instance which stores the objects name ('name'), name
	 * of the object's class ('object_class'), a list of values ('values'). Each value is a map specifying the attribute name
	 * ('attribute') and its value in a string rep form ('value').
	 * @param objects the list of OO-MDP object instances
	 * @return and OO-MDP {@link State} object.
	 */
	protected State JSONPreparedToState(List<Map<String, Object>> objects){

		State s = new State();

		for(Map<String, Object> oMap : objects){
			String obName = (String)oMap.get("name");
			String className = (String)oMap.get("object_class");
			ObjectInstance ob = new ObjectInstance(this.domain.getObjectClass(className), obName);

			List<Map<String, Object>> values = (List<Map<String, Object>>)oMap.get("values");
			for(Map<String, Object> v : values){
				String aname = (String)v.get("attribute");
				String vv = (String)v.get("value");
				ob.setValue(aname, vv);
			}

			s.addObject(ob);
		}

		return s;
	}


}
