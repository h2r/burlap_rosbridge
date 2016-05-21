package burlap.ros.actionpub;


import burlap.mdp.core.Action;
import ros.Publisher;
import ros.RosBridge;

/**
 * An interface for classes that can take as input an {@link Action}, turn it into
 * a ROS message and publish it to a ROS topic. The abstract class {@link burlap.ros.actionpub.ActionPublisher.DirectActionPublisher}
 * provides a {@link ros.Publisher} data member and constructors for easily setting up a means to publish to
 * {@link ros.RosBridge}.
 * <p>
 * Note that the {@link #publishAction(Action)} method should return a time delay, in
 * milliseconds, that tells the caller how long it should wait for the published action to finish executing. If the
 * {@link #publishAction(Action)} method blocks until an action has completed
 * executing on ROS, then it should return 0. This approach allows you to dynamically determine
 * if actions should be synchronous or asynchronous by how you implement {@link burlap.ros.actionpub.ActionPublisher}.
 * @author James MacGlashan.
 */
public interface ActionPublisher {


	/**
	 * Takes a BURLAP {@link Action} turns into a ROS message, and publishes it to ROS through this
	 * object's ROS Bridge {@link ros.Publisher} object. Note that when you implement this method, you must make the publish
	 * call to ROSBridge yourself using a {@link ros.Publisher} object.
	 * @param a The BURLAP {@link Action} to turn into a ROS message and publish.
	 * @return the time in milliseconds that the calling code should wait for the published action to finish executing. Will return 0 if this method blocks until ROS action completion.
	 */
	int publishAction(Action a);


	/**
	 * An abstract class that implements {@link burlap.ros.actionpub.ActionPublisher} and provides
	 * a {@link ros.Publisher} data member, constructors, and setters and getters for streamlining
	 * the publishing of messages to ROS BRidge. Note that you still need to implement
	 * the {@link burlap.ros.actionpub.ActionPublisher#publishAction(Action)} method
	 * yourself and within that method you will need to publish the created message to this objects {@link ros.Publisher}
	 * {@link #pub} data member or use this objects {@link #publish(Object)} method to indirectly publish to the data member.
	 */
	public static abstract class DirectActionPublisher implements ActionPublisher{

		/**
		 * The Rosbridge {@link ros.Publisher} used to publish action messages.
		 */
		protected Publisher pub;

		/**
		 * Initializes with a {@link ros.Publisher} for this action using the specified topic, message type, and {@link ros.RosBridge} connection.
		 * @param topic the ros topic to which messages will be published
		 * @param msgType the ros message type of th target topic
		 * @param rosBridge the {@link ros.RosBridge} connection to use.
		 */
		public DirectActionPublisher(String topic, String msgType, RosBridge rosBridge){
			this.pub = new Publisher(topic, msgType, rosBridge);
		}

		/**
		 * Initializes with a {@link ros.Publisher} for publishing action messages to ROS.
		 * @param pub
		 */
		public DirectActionPublisher(Publisher pub){
			this.pub = pub;
		}

		/**
		 * Returns the {@link ros.Publisher} used to publish action messages to ROS
		 * @return the {@link ros.Publisher} used to publish action messages to ROS
		 */
		public Publisher getPub() {
			return pub;
		}

		/**
		 * Sets the {@link ros.Publisher} used to publish action messages to ROS
		 * @param pub the {@link ros.Publisher} used to publish action messages to ROS
		 */
		public void setPub(Publisher pub) {
			this.pub = pub;
		}


		/**
		 * Publishes a ROS message specified by the msg argument via this object's {@link ros.Publisher} {@link #pub} data instances.
		 * Simply calls this.pub.publish(msg)
		 * @param msg the ROS message to publish
		 */
		public void publish(Object msg){
			this.pub.publish(msg);
		}

	}

}
