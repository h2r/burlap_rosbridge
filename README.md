burlap_rosbridge
================

Note that the master branch is now using the BURLAP 2 library. If you want ROS support for BURLAP version 1, use the v1 branch.

A BURLAP library extension for interacting with robots run on ROS by creating BURLAP `Environment` instances that maintain state and execution actions by ROS topics communicated via ROS Bridge.

Currently, there is an abstract `Environment`, `AbstractRosEnvironment` and one concrete implementation of it, `RosEnvironment` (there is also the `AsynchronousRosEnvironment` which is now deprecated). `AbstractRosEnvironment` provides the infrastructure for managing action execution (via the `Environment` `executeAction(GroundedAction)` method). In short, `AbstractRosEnvironment` allows you to specify `ActionPublisher` for each BURLAP `Action` that are responsible for publishing the action events to ROS. There are a number of included implementations of `ActionPublisher` in the library already, but the framework is made to enable you to implement your own. The way an `ActionPublisher` is implemented also affects whether action execution is synchronous or asynchronous.

The `AbstractRosEnvironment` does not, however, maintain the state of the Environment, which is a task left for the concrete implementations of it. The provided concrete implementation `RosEnvironment` adopts an approach to maintaining state by suscribing to a ROS topic that is publishing a ROS messages of type `burlap_msgs/burlap_state` that fully represents the BURLAP state. You can get the neccessary ROS message definition from the [burlap_msgs](https://github.com/h2r/burlap_msgs) project. This paradigm means that there must exist running ROS code that handles perception and turns it into a BURLAP state representation. If you need to do additional state processing not provided in the communicated ROS message (e.g., add additional "virutal" objects to the received state) you may do so by overriding the method `onStateReceive(State)` method of the `RosEnvironment` class (see its documentation for more information).

If you would prefer to have BURLAP code create the `State` from various standard ROS topics (rather than having a ROS topic that is publishing it), then you may want to create your own extension of `AbstractRosEnvironment` so that you can still beneift from the action publishing tools it provides. See its documentation for more information on how to do that, but so long as your implement its required abstract methods (and unimplemented methods inherited from `Environment`) it will work.

Although BURLAP is currently compatible with Java 6, You will need Java 7 to use this library because the ROS Bridge Websocket connection (provided by our [java_rosbridge](https://github.com/h2r/java_rosbridge) library) uses Jetty 9, which requires Java 7.

See the Java doc and example code below for more informaiton.

##Compiling

Compile with:

```
ant
```
Create a jar that you can use with other projects with:

```
ant dist
```

Alternatively, create a jar that includes the dependencies with 

```
ant dist_all
```

In both cases, the jar files will be stored in the `dist` folder.

Create java doc with:

```
ant doc
```

The produced Java doc will be in the `doc` folder.

profit.

##Example code
We provide two sets of example code. One is more straightforward for testing purposes. The latter shows you how to control a ROS robot that responds to Twist messages.

###Example 1
```
public static void main(String[] args) {

	//define the grid world
		GridWorldDomain gwd = new GridWorldDomain(11, 11);
		gwd.makeEmptyMap();
		final Domain domain = gwd.generateDomain();

		//setup ROS information
		String uri = "ws://localhost:9090";
		String stateTopic = "/burlap_state";
		String actionTopic = "/burlap_action";


		RosEnvironment env = new RosEnvironment(domain, uri, stateTopic);
		env.setActionPublisherForMultipleAcitons(domain.getActions(), new ActionStringPublisher(actionTopic, env.getRosBridge(), 500));

		//optionally, uncomment the below so that you can see the received state printed to the terminal
		//env.setPrintStateAsReceived(true);

		//create a random policy for control that connects to the environment wrapped domain
		Policy randPolicy = new RandomPolicy(domain);

		//begin behavior in the environment for 100 steps (50 seconds)
		randPolicy.evaluateBehavior(env, 100);

}

```


In this example code we assume that ROS is being run on the local host (port 9090 as default for ROS Bridge)
and that there is a ROS topic named `/burlap_state` that has a Grid World state message being published. For testing purposes, you can have ROS publish a dummy burlap_state message in which the agent is located at position 1,2 with the following command:

`rostopic pub /burlap_state burlap_msgs/burlap_state -r 1 -- '[{name: agent0, object_class: agent, values: [{attribute: x, value: "1"},{attribute: y, value: "2"}]}]'`

This command will cause the burlap_state to be published at a rate of 1hz. Naturally, since it is required for this code, you need to be using the burlap_msgs ROS package for the message types. These are available on [github](https://github.com/h2r/burlap_msgs) and are installed into your ROS workspace in the usual way. To confirm that your ROS workspace knows about the burlap_msg types, use the command `rosmsg list | grep "burlap"` which should print out the entires for burlap_state, burlap_object, and burlap_value. If they are not present and you installed (and compiled with catkin_make) the messages, you may need to re-source your ROS workspace with the command `source pathToWorkspace/devel/setup.bash`. You can confirm that your burlap_state message is indeed being published at 1hz using the rostopic command 

`rostopic echo /burlap_state`.

In this code, we have action execution for all actions handled by a single `ActionStringPublisher`. This implementation of `ActionPublisher` simply publishes the string representation of the input `GroundedAction` as a `std_msgs/String` ROS message to a designated topic (in this case, `/burlap_action`) and blocks further BURLAP executaion for some designated amount of time (in this case, 500 milliseconds) to allow the action on the robot to have an effect. For this publisher to actually do anything on a real robot, you would need ROS code that subscribed to `/burlap_action` and turned the string represetnation into a set of physical actuations (either through additional ROS topic publishing or by being the driver of the robot). Often times, you will probably want a more direct connection to the Robot's actuators than the `ActionStringPublisher` provides. However, for illustrative purposes this example is convenient because to see that the BURLAP-ROS connection is working, we simply need to run the ROS command 

`rostopic echo burlap_action`

which will print the strings received from BURLAP as actions are executed. Note that in this example code we simply have a random GridWorld policy running, so you should see a random assorment of "north," "south," "east," and "west."

###Example 2


