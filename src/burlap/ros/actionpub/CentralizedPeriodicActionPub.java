package burlap.ros.actionpub;

import burlap.oomdp.singleagent.GroundedAction;
import ros.tools.PeriodicPublisher;

/**
 * An {@link burlap.ros.actionpub.ActionPublisher} that flips the message being sent periodically by a {@link ros.tools.PeriodicPublisher}.
 * This is useful when you want a ROS message constantly published to a topic and want BURLAP actions to merely change what message
 * will be published in the future. For example, You might want to constantly publish a geometry_msgs/Twist message to robot
 * to define how its moving with the execution of different BURLAP actions canceling the previous movement directive for a new one.
 * Note that the provided {@link ros.tools.PeriodicPublisher} must have been started externally first with its
 * {@link ros.tools.PeriodicPublisher#beginPublishing(int)} or {@link ros.tools.PeriodicPublisher#beginPublishing(Object, int)} method.
 * If it has not been started then when this object gets a publish message, it will change the {@link ros.tools.PeriodicPublisher} message
 * to send, but it will never be sent because it hasn't been started.
 * @author James MacGlashan.
 */
public class CentralizedPeriodicActionPub implements ActionPublisher{

	/**
	 * The {@link ros.tools.PeriodicPublisher} whose message will be altered by this {@link burlap.ros.actionpub.ActionPublisher}.
	 */
	protected PeriodicPublisher ppub;

	/**
	 * The ROS message to which the provided {@link ros.tools.PeriodicPublisher} will be flipped when this objects {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method is called.
	 */
	protected Object msg;

	/**
	 * The time delay returned by the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method.
	 */
	protected int timeDelay;

	/**
	 * Initializes
	 * @param ppub the {@link ros.tools.PeriodicPublisher} whose message will be altered by this {@link burlap.ros.actionpub.ActionPublisher}.
	 * @param msg the ROS message to which the provided {@link ros.tools.PeriodicPublisher} will be flipped when this objects {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method is called.
	 * @param timeDelay the time delay returned by the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method.
	 */
	public CentralizedPeriodicActionPub(PeriodicPublisher ppub, Object msg, int timeDelay) {
		this.ppub = ppub;
		this.msg = msg;
		this.timeDelay = timeDelay;
	}


	/**
	 * Returns the {@link ros.tools.PeriodicPublisher} that has its message altered by this object.
	 * @return the {@link ros.tools.PeriodicPublisher} that has its message altered by this object.
	 */
	public PeriodicPublisher getPpub() {
		return ppub;
	}

	/**
	 * Sets the {@link ros.tools.PeriodicPublisher} that has its message altered by this object.
	 * @param ppub the {@link ros.tools.PeriodicPublisher} that has its message altered by this object.
	 */
	public void setPpub(PeriodicPublisher ppub) {
		this.ppub = ppub;
	}


	/**
	 * Returns the ROS message that the this object has periodically published
	 * @return the ROS message that the this object has periodically published
	 */
	public Object getMsg() {
		return msg;
	}

	/**
	 * Sets the ROS message that the this object has periodically published
	 * @param msg the ROS message that the this object has periodically published
	 */
	public void setMsg(Object msg) {
		this.msg = msg;
	}

	/**
	 * Returns the time delay in milliseconds returned by the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method
	 * @return the time delay in milliseconds returned by the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method
	 */
	public int getTimeDelay() {
		return timeDelay;
	}

	/**
	 * Sets the time delay in milliseconds returned by the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method
	 * @param timeDelay the time delay in milliseconds returned by the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method
	 */
	public void setTimeDelay(int timeDelay) {
		this.timeDelay = timeDelay;
	}

	@Override
	public int publishAction(GroundedAction a) {
		this.ppub.setMsg(msg);
		return this.timeDelay;
	}
}
