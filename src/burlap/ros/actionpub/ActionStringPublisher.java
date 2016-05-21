package burlap.ros.actionpub;

import burlap.mdp.core.Action;
import ros.Publisher;
import ros.RosBridge;
import ros.msgs.std_msgs.PrimitiveMsg;

/**
 * An {@link burlap.ros.actionpub.ActionPublisher} that publishes to ROS a std_msgs/String containing the string representation
 * of the {@link Action} to be published. The {@link #publishAction(Action)}
 * method will return a constant time delay specified by the constructor of this class.
 * @author James MacGlashan.
 */
public class ActionStringPublisher extends ActionPublisher.DirectActionPublisher{

	/**
	 * The time delay to return
	 */
	protected int delayTime;

	/**
	 * Initializes
	 * @param topic the ROS topic which should be of type std_msgs/String to which actions will be published.
	 * @param rosBridge the {@link ros.RosBridge} connection
	 * @param delayTime the delay time the {@link #publishAction(Action)} method will return.
	 */
	public ActionStringPublisher(String topic, RosBridge rosBridge, int delayTime) {
		super(topic, "std_msgs/String", rosBridge);
		this.delayTime = delayTime;
	}

	/**
	 * Initializes
	 * @param pub the std_msgs/String message type publisher to use
	 * @param delayTime the delay time the {@link #publishAction(Action)} method will return.
	 */
	public ActionStringPublisher(Publisher pub, int delayTime) {
		super(pub);
		this.delayTime = delayTime;
	}

	public int getDelayTime() {
		return delayTime;
	}

	public void setDelayTime(int delayTime) {
		this.delayTime = delayTime;
	}

	@Override
	public int publishAction(Action a) {
		PrimitiveMsg<String> msg =new PrimitiveMsg<String>(a.toString());
		this.publish(msg);
		return this.delayTime;
	}
}
