package burlap.ros;

import burlap.debugtools.DPrint;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.common.NullRewardFunction;
import burlap.oomdp.stateserialization.simple.SimpleSerializableState;
import com.fasterxml.jackson.databind.JsonNode;
import ros.MessageUnpacker;
import ros.RosListenDelegate;

/**
 * An implementation of {@link burlap.ros.AbstractRosEnvironment} that maintains the environment state by receiving burlap_msgs/burlap_state
 * ROS messages over ROSBridge.
 * <br/><br/>
 * Note that in the constructor you may want to set a low (e.g., 1) throttle rate and queue rate if burlap_msgs/burlap_state messages
 * are sent frequently, otherwise ROS Bridge may start lagging.
 * <br/><br/>
 * This class will block on the {@link #getCurrentObservation()} method until it receives a state message from
 * ROS Bridge. If you would like, you can override waiting for the first state message received from ROS and force this environment to report
 * a specific BURLAP {@link burlap.oomdp.core.states.State} using the {@link #overrideFirstReceivedState(burlap.oomdp.core.states.State)} method. Note
 * that any subsequent burlap_msgs/burlap_state messages will still update this environment's current state.
 * <br/><br/>
 * This class may also be provided a BURLAP {@link burlap.oomdp.singleagent.RewardFunction} and {@link burlap.oomdp.core.TerminalFunction}
 * for generating non-zero rewards and terminal states. Use the {@link #setRewardFunction(burlap.oomdp.singleagent.RewardFunction)}
 * and {@link #setTerminalFunction(burlap.oomdp.core.TerminalFunction)} to set them use the {@link #setRewardFunction(burlap.oomdp.singleagent.RewardFunction)}
 * and {@link #setTerminalFunction(burlap.oomdp.core.TerminalFunction)} methods.
 * <br/><br/>
 * If you would like to augment or further process the BURLAP {@link burlap.oomdp.core.states.State} that is parsed
 * from the ROS message, you can intercept it by overriding the {@link #onStateReceive(burlap.oomdp.core.states.State)}
 * method, and returning a different processed {@link burlap.oomdp.core.states.State}.
 * @author James MacGlashan.
 */
public class RosEnvironment extends AbstractRosEnvironment implements RosListenDelegate{


	/**
	 * The BURLAP {@link burlap.oomdp.core.Domain} into which states will be parsed
	 */
	protected Domain domain;

	/**
	 * The current {@link burlap.oomdp.core.states.State} representation of the environment
	 */
	protected State curState;

	/**
	 * The optional {@link burlap.oomdp.singleagent.RewardFunction} used to generate reward signals
	 */
	protected RewardFunction rf = new NullRewardFunction();

	/**
	 * The optional {@link burlap.oomdp.core.TerminalFunction} used to specify terminal states of the environment
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
	 * needing to be published to ROS. State information
	 * from ROS is expected to use be of
	 * type burlap_msgs/burlap_state. The burlap_state message is parsed into an actual BURLAP
	 * {@link burlap.oomdp.core.states.State} object using the object classes defined in a provided
	 * BURLAP {@link burlap.oomdp.core.Domain}.
	 * <br/>
	 * Remember that for actions to be properly handled, you will need to set the {@link burlap.ros.actionpub.ActionPublisher}
	 * to use for each action after this class is constructed with one of the appropriate methods (e.g., {@link #setActionPublisher(String, burlap.ros.actionpub.ActionPublisher)}).
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param rosBridgeURI the URI of the ros bridge server. Note that by default, ros bridge uses port 9090. An example URI is ws://localhost:9090
	 * @param rosStateTopic the name of the ROS topic that publishes the burlap_msgs/burlap_state messages.
	 */
	public RosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic){
		super(rosBridgeURI);
		this.domain = domain;
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
	 * Remember that for actions to be properly handled, you will need to set the {@link burlap.ros.actionpub.ActionPublisher}
	 * to use for each action after this class is constructed with one of the appropriate methods (e.g., {@link #setActionPublisher(String, burlap.ros.actionpub.ActionPublisher)}).
	 * @param domain the domain into which ROS burlap_state messages are parsed
	 * @param rosBridgeURI the URI of the ros bridge server. Note that by default, ros bridge uses port 9090. An example URI is ws://localhost:9090
	 * @param rosStateTopic the name of the ROS topic that publishes the burlap_msgs/burlap_state messages.
	 * @param rosBridgeThrottleRate the ROS Bridge server throttle rate: how frequently the server will send state messages
	 * @param rosBridgeQueueLength the ROS Bridge queue length: how many messages are queued on the server; queueing is a consequence of the throttle rate
	 */
	public RosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic, int rosBridgeThrottleRate, int rosBridgeQueueLength){
		super(rosBridgeURI);
		this.domain = domain;
		this.rosBridge.subscribe(rosStateTopic, "burlap_msgs/burlap_state", this, rosBridgeThrottleRate, rosBridgeQueueLength);

	}

	/**
	 * Creates an environment wrapper for state information provided over ROS with BURLAP actions
	 * needing to be published to ROS. This constructor allows you to specify
	 * the message type of the state topic if for some reason if the message type is not named
	 * the standard "burlap_msgs/burlap_state". However, this class will operate under the assumption
	 * that the message type adheres to the burlap_msgs/burlap_state structure. If it does not,
	 * then you will need to override the {@link #receive(com.fasterxml.jackson.databind.JsonNode, String)}
	 * method for parsing the BURLAP {@link burlap.oomdp.core.states.State} object out of the ROS message.
	 * <br/>
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
	public RosEnvironment(Domain domain, String rosBridgeURI, String rosStateTopic, String rosStateMessageType, int rosBridgeThrottleRate, int rosBridgeQueueLength){
		super(rosBridgeURI);
		this.domain = domain;
		this.rosBridge.subscribe(rosStateTopic, rosStateMessageType, this, rosBridgeThrottleRate, rosBridgeQueueLength);

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
		this.blockUntilStateReceived();
		return this.curState.copy();
	}


	@Override
	public boolean isInTerminalState() {
		return this.tf.isTerminal(this.curState);
	}


	@Override
	protected double getMostRecentRewardSignal(State s, GroundedAction ga, State sprime) {
		return this.rf.reward(s, ga, sprime);
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


	@Override
	public void receive(JsonNode data, String stringRep) {

		MessageUnpacker <SimpleSerializableState> unpacker = new MessageUnpacker(SimpleSerializableState.class);
		SimpleSerializableState sss = unpacker.unpackRosMessage(data);
		State s = sss.deserialize(this.domain);
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



}
