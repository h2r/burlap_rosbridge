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
In the last example we setup an environment that published actions as string representations of the action name. This approach is only effective if you have some ROS code running that knows how to interpret the string representations
and actuate it on the robot, thereby requiring a middle man. A more direct way to control the robot is to have
action execution publish more typical ROS messages that specify the actuation. For example, on turtlebot robots, and various other robots, it is common to publish a ROS Twist message over a period of time to have the robot move. In this example code, we setup the BURLAP environment so that action execution results in publishing a Twist message for a fixed duration, thereby directly controlling the robot without a middle-man ROS script. We also then use a `TerminalExplorer` over the Environment we created so that you can manually control the robot through the terminal to test it out. 

```
public static void main(String [] args){

	//create a new domain with no state representation
	Domain domain = new SADomain();

	//create action specification (we don't need to define transition dynamics since we won't be doing any planning)
	new NullAction("forward", domain); //forward
	new NullAction("backward", domain); //backward
	new NullAction("rotate", domain); //clockwise rotate
	new NullAction("rotate_ccw", domain); //counter-clockwise rotate


	//setup ROS information
	String uri = "ws://localhost:9090";
	String stateTopic = "/burlap_state"; //we won't need this in this example, so set it to anything
	String actionTopic = "/mobile_base/commands/velocity"; //set this to the appropriate topic for your robot!
	String actionMsg = "geometry_msgs/Twist";


	//define the relevant twist messages that we'll use for our actions
	Twist fTwist = new Twist(new Vector3(0.1,0,0.), new Vector3()); //forward
	Twist bTwist = new Twist(new Vector3(-0.1,0,0.), new Vector3()); //backward
	Twist rTwist = new Twist(new Vector3(), new Vector3(0,0,-0.5)); //clockwise rotate
	Twist rccwTwist = new Twist(new Vector3(), new Vector3(0,0,0.5)); //counter-clockwise rotate

	//create environment
	RosEnvironment env = new RosEnvironment(domain, uri, stateTopic);

	int period = 500; //publish every 500 milliseconds...
	int nPublishes = 5; //...for 5 times for each action execution...
	boolean sync = true; //...and use synchronized action execution
	env.setActionPublisher("forward", new RepeatingActionPublisher(actionTopic, actionMsg, env.getRosBridge(), fTwist, period, nPublishes, sync));
	env.setActionPublisher("backward", new RepeatingActionPublisher(actionTopic, actionMsg, env.getRosBridge(), bTwist, period, nPublishes, sync));
	env.setActionPublisher("rotate", new RepeatingActionPublisher(actionTopic, actionMsg, env.getRosBridge(), rTwist, period, nPublishes, sync));
	env.setActionPublisher("rotate_ccw", new RepeatingActionPublisher(actionTopic, actionMsg, env.getRosBridge(), rccwTwist, period, nPublishes, sync));

	//force the environment state to a null state so we don't have to setup a burlap_state topic on ROS
	env.overrideFirstReceivedState(new MutableState());


	//create a terminal controlled explorer to run on our environment
	//so that we can control the robot with the keyboard
	TerminalExplorer exp = new TerminalExplorer(domain, env);

	//add some alias for the action names so that we don't have to write the full action name out
	exp.addActionShortHand("d", "rotate");
	exp.addActionShortHand("a", "rotate_ccw");
	exp.addActionShortHand("w", "forward");
	exp.addActionShortHand("s", "backward");
	exp.addActionShortHand("x", "stop");

	exp.explore();

}
```
The first thing to note about this example code is that we are not using GridWorld. For this example, we don't care to maintain any state since we simply want to show you how to use the code to directly control a robot with Twist messages. Therefore, we create an empty BURLAP `Domain` with some `Action` objects for a forward, backward, and rotate actions (we use the `NullAction` since we are ony going to execute actions through the `RosEnvironment` and therefore don't need to define any transition dynamics).

Next we create Twist objects that are part of the [java_rosbridge](https://github.com/h2r/java_rosbridge) library we are using. These objects are Java Beans that follow the same structure as the ROS Twist message: a linear `Vector3` component and an angular `Vector3` component. Note that if you were writing code for a different kind of robot that didn't respond to Twist messages and used a message type not in [java_rosbridge](https://github.com/h2r/java_rosbridge), you could simply create your own [Java Bean](https://en.wikipedia.org/wiki/JavaBeans) class for the ROS message and use that just as well.

For the ROS environment we have to provide standard information about the ROS Bridge URI and the state topic. We won't actually be using the state topic in this example, so we could give it any name.

After creating our `RosEnvironment` we set up a `ActionPublisher` for each of our BURLAP actions. Specifically, we use a `RepeatingActionPublisher`. A `RepeatingActionPublisher` will have the affect of publishing a specified message a fixed number of times at a specified period. Specifically, for each BURLAP action, we define a `RepeatingActionPublisher` that will publish the corresponding Twist message 5 times with a period of 500 milliseconds between each publish. We also have to specify the action topic for this message. We set it to the topic commonly used by the Turtlebot robot, but you should be sure set it to whichever topic your Twist-controlled robot will respond. Note that we set a synchronized flag for the  `RepeatingActionPublisher` publisher to true. This has the effect of the `ActionPublisher` blocking until it has published all 5 messages before returning control to the calling `Environment`. Because we set it to be synchronized, it will also automatically set the return delay to the period length, which will cause the `RosEnvironment` to wait an additional 500 millseconds after the `RepeatingActionPublisher` published its final 5th message thereby allowing time for that final message to have an affect.

Normally when interacting with a `RosEnvironment` no observations or actions will be permitted until it receives from ROS the first State message (indicating the initial state of the environment). In this example, however, we are not going to write any state generation code, so no message will ever arrive. Therefore, we use the `env.overrideFirstReceivedState(new MutableState())` call to force the Environment to think the current state is the one provided (an empty `State`) so that we can continue without waiting for a message that will never come.

Finally, we use the standard BURLAP `TerminalExplorer` to allow us to manually interact with the `Environment` with keyboard commands. We also give some action name alias of w,a,s, and d so that we can simply type that key (and press enter) to have it execute a forward, rotate, or backward action. If you run the code, you should find that when you enter the keyboard commands it causes the robot to move.


### Custom State Messages

In the above examples, we talked about how to use `RosEnvironment` which subscribes to a ROS topic that is expected to publish BURLAP states via the message type `burlap_msgs/burlap_state`. However, in some cases you may want to subcribe to one or more different topics and build the BURLAP `State` from them in Java. If you wish to do that, then you will either want to extend `AbstractRosEnviroment` which includes the code for managing action publishing, but does not include any code for building the state (see its documentation for more information); or if the state is fully defined by one topic, you may wish to subclass `RosEnvironment` and override the method `unpackStateFromMsg(JsonNode data, String stringRep)` which is the method that handles parsing the JSON message from ROS into a BURLAP state (see its documentation for more information).

#### Large Message Sizes

If you're building your state from a custom message, it's possible the message you're receiving from ROS is very large, such as frames from a video feed. In these cases, it's likely that the message size is larger than what Jetty's websocket buffer size is by default. However, you can increase the buffer size by subclassing `RosBridge` and annotating the subclass to have a larger buffer size. You do not need to implement or override any methods; `RosBridge` is subclassed purely to give it a custom buffer size annotation. For example:

```
@WebSocket(maxTextMessageSize = 500 * 1024) public class BigMessageRosBridge extends RosBridge{}
```

If you then instantiate your subclass, connect with it, and use a AbstractRosEnvironment or RosEnvironment constructor that accepts a RosBridge instance (rather than one that asks for the URI), then your environment will be able to handle the larger message sizes. See the readme and Java doc for [java_rosbridge](https://github.com/h2r/java_rosbridge) for more information.
