package burlap.ros;

import burlap.debugtools.DPrint;
import burlap.mdp.auxiliary.common.NullTermination;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.common.NullRewardFunction;
import burlap.mdp.singleagent.model.RewardFunction;
import com.fasterxml.jackson.databind.JsonNode;
import ros.RosBridge;
import ros.RosListenDelegate;

/**
 * An abstract implementation of {@link burlap.ros.AbstractRosEnvironment} that maintains the environment state by receiving
 * a single ROS message corresponding to the state of the environment. To implement this class, implement the {@link #unpackStateFromMsg(JsonNode, String)}
 * method to provide the code that parses the ROS message into a BURLAP {@link burlap.mdp.core.state.State}.
 * <p>
 * Note that in the constructor you may want to set a low (e.g., 1) throttle rate and queue rate if state messages
 * are sent frequently, otherwise ROS Bridge may start lagging.
 * <p>
 * This class will block on the {@link #currentObservation()} method until it receives a state message from
 * ROS Bridge. If you would like, you can override waiting for the first state message received from ROS and force this environment to report
 * a specific BURLAP {@link State} using the {@link #overrideFirstReceivedState(State)} method. Note
 * that any subsequent burlap_msgs/burlap_state messages will still update this environment's current state.
 * <p>
 * This class may also be provided a BURLAP {@link RewardFunction} and {@link TerminalFunction}
 * for generating non-zero rewards and terminal states. Use the {@link #setRewardFunction(RewardFunction)}
 * and {@link #setTerminalFunction(TerminalFunction)} to set them use the {@link #setRewardFunction(RewardFunction)}
 * and {@link #setTerminalFunction(TerminalFunction)} methods. This class does not have the {@link #handleEnterTerminalState()}
 * inherited from {@link burlap.ros.AbstractRosEnvironment} do anything, so if you want to inject code for handling the event
 * when the environment enters a terminal state, you should subclass this class and override that method.
 * @author James MacGlashan.
 */
public abstract class RosEnvironment extends AbstractRosEnvironment implements RosListenDelegate{


	/**
	 * The BURLAP {@link SADomain} into which states will be parsed
	 */
	protected SADomain domain;

	/**
	 * The current {@link State} representation of the environment
	 */
	protected State curState;

	/**
	 * The optional {@link RewardFunction} used to generate reward signals
	 */
	protected RewardFunction rf = new NullRewardFunction();

	/**
	 * The optional {@link TerminalFunction} used to specify terminal states of the environment
	 */
	protected TerminalFunction tf = new NullTermination();


	/**
	 * Indicates whether the first state message from ROS has been received yet
	 */
	protected Boolean receivedFirstState = false;


	/**
	 * Debug flag indicating whether states should be printed to the terminal as they are received. Default value is false.
	 */
	protected boolean printStateAsReceived = false;


	/**
	 * The debug code used for debug prints.
	 */
	protected int debugCode = 7345252;


	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS.
	 * <p>
	 * Remember that for actions to be properly handled, you will need to set the {@link burlap.ros.actionpub.ActionPublisher}
	 * to use for each action after this class is constructed with one of the appropriate methods (e.g., {@link #setActionPublisher(String, burlap.ros.actionpub.ActionPublisher)}).
	 *
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param rosBridgeURI the URI of the ros bridge server. Note that by default, ros bridge uses port 9090. An example URI is ws://localhost:9090
	 * @param rosStateTopic the name of the ROS topic that publishes the state messages.
	 * @param rosStateMessageType the message type of the ROS state messages.
	 */
	public RosEnvironment(SADomain domain, String rosBridgeURI, String rosStateTopic, String rosStateMessageType){
		this(domain, rosBridgeURI, rosStateTopic, rosStateMessageType, 1, 1);

	}


	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS.
	 * <p>
	 * Remember that for actions to be properly handled, you will need to set the {@link burlap.ros.actionpub.ActionPublisher}
	 * to use for each action after this class is constructed with one of the appropriate methods (e.g., {@link #setActionPublisher(String, burlap.ros.actionpub.ActionPublisher)}).
	 *
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param rosBridgeURI the URI of the ros bridge server. Note that by default, ros bridge uses port 9090. An example URI is ws://localhost:9090
	 * @param rosStateTopic the name of the ROS topic that publishes the state messages.
	 * @param rosStateMessageType the message type of the ROS state messages.
	 * @param rosBridgeThrottleRate the ROS Bridge server throttle rate: how frequently the server will send state messages
	 * @param rosBridgeQueueLength the ROS Bridge queue length: how many messages are queued on the server; queueing is a consequence of the throttle rate
	 */
	public RosEnvironment(SADomain domain, String rosBridgeURI, String rosStateTopic, String rosStateMessageType, int rosBridgeThrottleRate, int rosBridgeQueueLength){
		super(rosBridgeURI);
		this.domain = domain;
		this.rosBridge.subscribe(rosStateTopic, rosStateMessageType, this, rosBridgeThrottleRate, rosBridgeQueueLength);

	}


	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS. The state message type is assumed to be burlap_msgs/burlap_state.
	 * <p>
	 * Remember that for actions to be properly handled, you will need to set the {@link burlap.ros.actionpub.ActionPublisher}
	 * to use for each action after this class is constructed with one of the appropriate methods (e.g., {@link #setActionPublisher(String, burlap.ros.actionpub.ActionPublisher)}).
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param ros the connected {@link RosBridge} instance.
	 * @param rosStateTopic the name of the ROS topic that publishes the burlap_msgs/burlap_state messages.
	 * @param rosStateMessageType the message type of the ROS state messages.
	 */
	public RosEnvironment(SADomain domain, RosBridge ros, String rosStateTopic, String rosStateMessageType){
		super(ros);
		this.domain = domain;
		this.rosBridge.subscribe(rosStateTopic, rosStateMessageType, this);



	}

	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS. The message type is assumed to be burlap_msgs/burlap_state.
	 * <p>
	 * Remember that for actions to be properly handled, you will need to set the {@link burlap.ros.actionpub.ActionPublisher}
	 * to use for each action after this class is constructed with one of the appropriate methods (e.g., {@link #setActionPublisher(String, burlap.ros.actionpub.ActionPublisher)}).
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param ros the connected {@link RosBridge} instance.
	 * @param rosStateTopic the name of the ROS topic that publishes the burlap_msgs/burlap_state messages.
	 * @param rosBridgeThrottleRate the ROS Bridge server throttle rate: how frequently the server will send state messages
	 * @param rosBridgeQueueLength the ROS Bridge queue length: how many messages are queued on the server; queueing is a consequence of the throttle rate
	 */
	public RosEnvironment(SADomain domain, RosBridge ros, String rosStateTopic, int rosBridgeThrottleRate, int rosBridgeQueueLength){
		super(ros);
		this.domain = domain;
		this.rosBridge.subscribe(rosStateTopic, "burlap_msgs/burlap_state", this, rosBridgeThrottleRate, rosBridgeQueueLength);

	}

	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS. Here, the state message type may be specified to be something other than
	 * burlap_msgs/burlap_state. Note that unless you have subclassed
	 * {@link RosEnvironment} and overridden {@link #unpackStateFromMsg(JsonNode, String)}, the message type should be a type
	 * that is or adheres to "burlap_msgs/burlap_state" even if named something else.
	 * <p>
	 * Remember that for actions to be properly handled, you will need to set the {@link burlap.ros.actionpub.ActionPublisher}
	 * to use for each action after this class is constructed with one of the appropriate methods (e.g., {@link #setActionPublisher(String, burlap.ros.actionpub.ActionPublisher)}).
	 *
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param ros the connected {@link RosBridge} instance.
	 * @param rosStateTopic the name of the ROS topic that publishes the state messages.
	 * @param rosStateMessageType the message type of the ROS state messages.
	 * @param rosBridgeThrottleRate the ROS Bridge server throttle rate: how frequently the server will send state messages
	 * @param rosBridgeQueueLength the ROS Bridge queue length: how many messages are queued on the server; queueing is a consequence of the throttle rate
	 */
	public RosEnvironment(SADomain domain, RosBridge ros, String rosStateTopic, String rosStateMessageType, int rosBridgeThrottleRate, int rosBridgeQueueLength){
		super(ros);
		this.domain = domain;
		this.rosBridge.subscribe(rosStateTopic, rosStateMessageType, this, rosBridgeThrottleRate, rosBridgeQueueLength);

	}


	/**
	 * Sets a BURLAP {@link RewardFunction} to use to provide reward signals from the {@link #lastReward()} method.
	 * @param rf {@link RewardFunction} to use to provide reward signals.
	 */
	public void setRewardFunction(RewardFunction rf){
		this.rf = rf;
	}

	/**
	 * Sets the BURLAP {@link TerminalFunction} to use to provide terminal state checks from the {@link #isInTerminalState()}.
	 * @param tf the BURLAP {@link TerminalFunction} to use to provide terminal state checks
	 */
	public void setTerminalFunction(TerminalFunction tf){
		this.tf = tf;
	}


	/**
	 * Returns the domain of this environment.
	 * @return the domain of this environment.
	 */
	public SADomain getDomain() {
		return domain;
	}

	@Override
	public State currentObservation() {
		this.blockUntilStateReceived();
		return this.curState.copy();
	}



	public boolean isInTerminalState() {
		return this.tf.isTerminal(this.curState);
	}



	protected double getMostRecentRewardSignal(State s, Action a, State sprime) {
		return this.rf.reward(s, a, sprime);
	}

	/**
	 * Sets whether the default implementation of {@link #onStateReceive(State)} will print the
	 * state information to the terminal.
	 * @param printStateAsReceived if true, then the default implementation of {@link #onStateReceive(State)} will print states to the screen;
	 *                             if false, the state receiving is silent by default.
	 */
	public void setPrintStateAsReceived(boolean printStateAsReceived){
		this.printStateAsReceived = printStateAsReceived;
	}


	/**
	 * Use this method to override the first received state from ROS to be the provided {@link State}.
	 * If the {@link #blockUntilStateReceived()} method has been called in another thread, it will be notified that the first state is received.
	 * Note that if ROS Bridge subsequently sends state message, the state of this environment will be updated to reflect them.
	 * @param s the {@link State} to force this Environment to.
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
		if(!this.receivedFirstState) {
			DPrint.cl(this.debugCode, "Blocking until state received.");
		}
		boolean oldReceived = this.receivedFirstState;
		while(!this.receivedFirstState){
			try {
				this.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		if(!oldReceived) {
			DPrint.cl(this.debugCode, "State received");
		}

	}



	public void receive(JsonNode data, String stringRep) {

		State s = this.unpackStateFromMsg(data, stringRep);
		this.curState = this.onStateReceive(s);

		if(!this.receivedFirstState){
			synchronized (this){
				this.receivedFirstState = true;
				this.notifyAll();
			}
		}

	}


	/**
	 * This method receive the RosBridge message and unpacks it into a BURLAP {@link State} object.
	 * @param data the JSON message.
	 * @param stringRep the string representation of the JSON message.
	 * @return a {@link State} object that is unpacked from the JSON message.
	 */
	public abstract State unpackStateFromMsg(JsonNode data, String stringRep);


	/**
	 * This method is called after a {@link State} is unpacked from a ROS message. The new current state of the environment
	 * will be set to whatever this method returns. By default, this method simply returns the same reference and
	 * if this environment's {@link #printStateAsReceived} data member is set to true, then it will print
	 * to the terminal the string representation of the state. Override this method to provide
	 * special handling of used state (e.g., adding virtual objects to the state that ROS does not perceive).
	 * @param s the parsed state from the ROS message received.
	 * @return returns the {@link State} object parsed from the ros message
	 */
	protected State onStateReceive(State s){
		if(printStateAsReceived) {
			System.out.println(s.toString() + "\n-------------------------");
		}
		return s;
	}

	@Override
	protected void handleEnterTerminalState() {
		//do nothing
	}
}
