package burlap.ros.actionpub;

import burlap.oomdp.singleagent.GroundedAction;
import ros.Publisher;
import ros.RosBridge;

import java.util.Timer;
import java.util.TimerTask;

/**
 * An {@link burlap.ros.actionpub.ActionPublisher} that allows a single call to {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)}
 * to publish a ROS message multiple times at a given rate. This is useful, for example, if to execute a complete "rotate" action
 * a geometry_msgs/Twist message needs to be published to a robot multiple times in quick succession. This class can either
 * be synchronous, in which the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method blocks until
 * all repeated publishes are complete; or asynchronous, in which the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method
 * returns immediately after starting its multiple publish calls in a separate thread.
 * <br/><br/>
 * Note that by default, the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method will return a delay time
 * of zero if set to asynchronous, or the period is set to synchronous.
 * However, you can change this value with the {@link #setDelayTime(int)} method or by using the more complete constructor.
 * @author James MacGlashan.
 */
public class RepeatingActionPublisher extends ActionPublisher.DirectActionPublisher {


	/**
	 * The ROS message to publish
	 */
	protected Object msg;


	/**
	 * The length of time in milliseconds between publish messages
	 */
	protected int period;

	/**
	 * The number of times a message will be published for each call of {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)}
	 */
	protected int n;

	/**
	 * If true, then {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} blocks until all messages
	 * have been published. If false, then the method immediately returns after starting publishing
	 */
	protected boolean synchronous;

	/**
	 * The time delay to return
	 */
	protected int delayTime = 0;


	/**
	 * Initializes. If synchronous is set to true, the returned delay time by {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} will
	 * be set to the same as the period.
	 * @param topic the ROS topic to publish to
	 * @param msgType the ROS message type of the ROS topic
	 * @param rosBridge the {@link ros.RosBridge} connection
	 * @param msg the constant ROS message that will always be published
	 * @param period the time delay between subsequent publishes to ros
	 * @param n the number of times a ros message will be published for each call of {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)}
	 * @param synchronous if true, {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} returns immediately; if false, waits for all n publishes to complete.
	 */
	public RepeatingActionPublisher(String topic, String msgType, RosBridge rosBridge, Object msg, int period, int n, boolean synchronous) {
		super(topic, msgType, rosBridge);
		this.msg = msg;
		this.period = period;
		this.n = n;
		this.synchronous = synchronous;
		if(synchronous){
			this.delayTime = period;
		}
	}

	/**
	 * Initializes
	 * @param topic the ROS topic to publish to
	 * @param msgType the ROS message type of the ROS topic
	 * @param rosBridge the {@link ros.RosBridge} connection
	 * @param msg the constant ROS message that will always be published
	 * @param period the time delay between subsequent publishes to ros
	 * @param n the number of times a ros message will be published for each call of {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)}
	 * @param synchronous if true, {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} returns immediately; if false, waits for all n publishes to complete.
	 * @param delayTime the time to delay returned by {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)}.
	 */
	public RepeatingActionPublisher(String topic, String msgType, RosBridge rosBridge, Object msg, int period, int n, boolean synchronous, int delayTime) {
		super(topic, msgType, rosBridge);
		this.msg = msg;
		this.period = period;
		this.n = n;
		this.synchronous = synchronous;
		this.delayTime = delayTime;
	}

	/**
	 * Initializes
	 * @param pub the {@link ros.Publisher} used to publish action messages.
	 * @param msg the constant ROS message that will always be published
	 * @param period the  time delay between subsequent publishes to ros
	 * @param n the number of times a ros message will be published for each call of {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)}
	 * @param synchronous if true, {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} returns immediately; if false, waits for all n publishes to complete.
	 */
	public RepeatingActionPublisher(Publisher pub, Object msg, int period, int n, boolean synchronous) {
		super(pub);
		this.msg = msg;
		this.period = period;
		this.n = n;
		this.synchronous = synchronous;
	}

	/**
	 * Initializes
	 * @param pub the {@link ros.Publisher} used to publish action messages.
	 * @param msg the constant ROS message that will always be published
	 * @param period the  time delay between subsequent publishes to ros
	 * @param n the number of times a ros message will be published for each call of {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)}
	 * @param synchronous if true, {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} returns immediately; if false, waits for all n publishes to complete.
	 * @param delayTime the time to delay returned by {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)}.
	 */
	public RepeatingActionPublisher(Publisher pub, Object msg, int period, int n, boolean synchronous, int delayTime) {
		super(pub);
		this.msg = msg;
		this.period = period;
		this.n = n;
		this.synchronous = synchronous;
		this.delayTime = delayTime;
	}

	public Object getMsg() {
		return msg;
	}

	public void setMsg(Object msg) {
		this.msg = msg;
	}

	public int getPeriod() {
		return period;
	}

	public void setPeriod(int period) {
		this.period = period;
	}

	public int getN() {
		return n;
	}

	public void setN(int n) {
		this.n = n;
	}

	public boolean isSynchronous() {
		return synchronous;
	}

	public void setSynchronous(boolean synchronous) {
		this.synchronous = synchronous;
	}

	/**
	 * Returns the value that the method {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} will return.
	 * @return the value that the method {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} will return.
	 */
	public int getDelayTime() {
		return delayTime;
	}

	/**
	 * By default, {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} will return zero; use this method
	 * to set an alternative value for it to return.
	 * @param delayTime the value that the method {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} will return.
	 */
	public void setDelayTime(int delayTime) {
		this.delayTime = delayTime;
	}

	@Override
	public int publishAction(GroundedAction a) {
		Timer timer = new Timer();
		PublishTask pt = new PublishTask();
		timer.schedule(pt, 0, this.period);
		if(this.synchronous){
			synchronized(pt) {
				while(!pt.finished()) {
					try {
						pt.wait();
					} catch(InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		}

		return this.delayTime;
	}


	/**
	 * Callback class for timer event that publishes the ROS message every time it is called
	 * until it's been called for all initial n times specified.
	 */
	protected class PublishTask extends TimerTask{

		int nRemain;

		public PublishTask() {
			this.nRemain = n;
		}

		public boolean finished(){
			return nRemain <= 0;
		}

		@Override
		public void run() {
			publish(msg);
			this.nRemain--;
			if(nRemain <= 0){
				this.cancel();
				synchronized(this) {
					this.notifyAll();
				}
			}
		}
	}
}
