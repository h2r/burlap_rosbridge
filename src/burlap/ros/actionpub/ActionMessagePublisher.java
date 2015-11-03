package burlap.ros.actionpub;

import burlap.oomdp.singleagent.GroundedAction;
import ros.Publisher;
import ros.RosBridge;

/**
 * An {@link burlap.ros.actionpub.ActionPublisher} that publishes to ROS a fixed ROS message everytime
 * {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} is called. The {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)}
 * method will return a constant time delay specified by the constructor of this class.
 * @author James MacGlashan.
 */
public class ActionMessagePublisher extends ActionPublisher.DirectActionPublisher {

	/**
	 * The ROS message to publish
	 */
	protected Object msg;

	/**
	 * The time delay to return
	 */
	protected int delayTime;


	/**
	 * Initializes
	 * @param topic the ROS topic to publish to
	 * @param msgType the ROS message type of the ROS topic
	 * @param rosBridge the {@link ros.RosBridge} connection
	 * @param msg the constant ROS message that will always be published
	 * @param delayTime the delay time the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method will return.
	 */
	public ActionMessagePublisher(String topic, String msgType, RosBridge rosBridge, Object msg, int delayTime) {
		super(topic, msgType, rosBridge);
		this.msg = msg;
		this.delayTime = delayTime;
	}

	/**
	 * Initializes
	 * @param pub the {@link ros.Publisher} used to publish action messages.
	 * @param msg the constant ROS message that will always be published
	 * @param delayTime the delay time the {@link #publishAction(burlap.oomdp.singleagent.GroundedAction)} method will return.
	 */
	public ActionMessagePublisher(Publisher pub, Object msg, int delayTime) {
		super(pub);
		this.msg = msg;
		this.delayTime = delayTime;
	}

	@Override
	public int publishAction(GroundedAction a) {
		this.publish(this.msg);
		return this.delayTime;
	}
}
