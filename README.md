burlap_rosbridge
================

A BURLAP library extension for interacting with robots run on ROS by creating BURLAP `Environment` instances that maintain state and execution actions by ROS topics communicated via ROS Bridge.

##Linking
burlap_rosbridge is indexed on Maven Central, so if you want to merely use it, all you need to do is include in the `<dependencies>` section of your project's pom.xml file:
```
<dependency>
  <groupId>edu.brown.cs.burlap</groupId>
  <artifactId>burlap_rosbridge</artifactId>
  <version>3.0.0</version>
</dependency>
```
and it will automatically be downloaded. (DISCLAIMER: as of writing this, BURLAP 3 and burlap_rosbridge 3 are in development, and are not yet on Maven Central; you will have to manually install both.) Note that you will also want to explicitly include BURLAP (also on Maven Central) because BURLAP is set to not link transitively through burlap_rosbridge. This choice was made to make it clear which version of BURLAP you wanted to use in your project. To link to BURLAP, add
```
<dependency>
  <groupId>edu.brown.cs.burlap</groupId>
  <artifactId>burlap</artifactId>
  <version>3.0.0</version>
</dependency>
```
or switch its version to whatever is appropriate.

Alternatively, you may compile and install the code directly (or modify as needed), as described in the compiling section of this readme.


##Description of Code
Currently, there are two abstract `Environment` implementations, `AbstractRosEnvironment` and a slightly more concrete implementation, `RosEnvironment`. `AbstractRosEnvironment` provides the infrastructure for managing action execution (via the `Environment` `executeAction(Action)` method). In short, `AbstractRosEnvironment` allows you to specify `ActionPublisher` objects for each BURLAP `Action` that are responsible for publishing the action events to ROS. There are a number of included implementations of `ActionPublisher` in the library already, but the framework is made to enable you to implement your own. The way an `ActionPublisher` is implemented also affects whether action execution is synchronous or asynchronous.

The `AbstractRosEnvironment` does not, however, maintain the state of the Environment, which is a task left for the concrete implementations of it. The `RosEnvironment` extends `AbstractRosEnvrionment` with additional methods in which there is a single ROS topic that is publishing the state of the environment. Rewards and termination conditions for the environment are then handled by specifying standard BURLAP RewardFunction and TerminalFunction implementations that will operate on the state. Implementing `RosEnvironment` requires implementing a single method: `State unpackStateFromMsg(JsonNode data, String stringRep)` which is given the JSON message of the ROS topic and should turn it into a BURLAP `State` object that is returned. You may want to use the provided `MessageUnpacker` class if you have an implemented `State` object whose datastructure matches the ROS message and can be straightforwardly unpacked.


If you would prefer to have BURLAP code create the `State` from various ROS topics (rather than having a single ROS topic that is publishing it), then you may want to create your own extension of `AbstractRosEnvironment`.

Although BURLAP is currently compatible with Java 6, You will need Java 7 to use this library because the ROS Bridge Websocket connection (provided by our [java_rosbridge](https://github.com/h2r/java_rosbridge) library) uses Jetty 9, which requires Java 7.

See the Java doc and example code below for more informaiton.

##Compiling

Compiling and management is performed with Maven. 

To compile use

```
mvn compile
```
Create a target jar and Java doc with

```
mvn package
```

Install the jar into your local repoistory with

```
mvn install
```

Link to burlap_rosbridge from a project by adding the following to the `<dependencies>` section of your project's pom.xml file.

```
<dependency>
  <groupId>edu.brown.cs.burlap</groupId>
  <artifactId>burlap_rosbridge</artifactId>
  <version>3.0.0</version>
</dependency>
```


##Example code
We provide two sets of example code. One is more straightforward for testing purposes. The latter shows you how to control a ROS robot that responds to Twist messages.

###Example 1
```
public class Example1 {

	public static class StringState implements State{

		public String data;

		public StringState() {
		}

		public StringState(String data) {
			this.data = data;
		}

		@Override
		public List<Object> variableKeys() {
			return Arrays.<Object>asList("data");
		}

		@Override
		public Object get(Object variableKey) {
			return data;
		}

		@Override
		public State copy() {
			return new StringState(data);
		}

		@Override
		public String toString() {
			return data;
		}
	}

	public static void main(String[] args) {

		SADomain domain = new SADomain();
		domain.addActionTypes(new UniversalActionType("action1"), new UniversalActionType("action2"));

		//setup ROS information
		String uri = "ws://localhost:9090";
		String stateTopic = "/burlap_state";
		String stateMessage = "std_msgs/String";
		String actionTopic = "/burlap_action";


		RosEnvironment env = new RosEnvironment(domain, uri, stateTopic, stateMessage) {
			@Override
			public State unpackStateFromMsg(JsonNode data, String stringRep) {
				MessageUnpacker<StringState> unpacker = new MessageUnpacker<StringState>(StringState.class);
				return unpacker.unpackRosMessage(data);
			}
		};
		env.setActionPublisherForMultipleAcitons(domain.getActionTypes(), new ActionStringPublisher(actionTopic, env.getRosBridge(), 500));
		env.setPrintStateAsReceived(true);

		Policy randPolicy = new RandomPolicy(domain);
		randPolicy.evaluateBehavior(env, 100);

	}

}

```


In this example code we assume that ROS is being run on the local host (port 9090 as default for ROS Bridge)
and that there is a ROS topic named `/burlap_state` that is defined by a single string state variable. For testing purposes, you can have ROS publish a dummy burlap_state message that simply has the string value "hello" with the following ros command:

`rostopic pub /burlap_state std_msgs/String -r 1 -- 'hello'`

The first thing we do is define a cooresponding BURLAP `State` definition for a state that is defined by a single string value. Note that to make unpacking trivial, the State class uses data fields that perfectly match the ROS data structure for the std_msgs/String message; that is, it is defined by a single String field named `data`.

In this code, we have action execution for all actions handled by a single `ActionStringPublisher`. This implementation of `ActionPublisher` simply publishes the string representation of the input `Action` as a `std_msgs/String` ROS message to a designated topic (in this case, `/burlap_action`) and blocks further BURLAP executaion for some designated amount of time (in this case, 500 milliseconds) to allow the action on the robot to have an effect. For this publisher to actually do anything on a real robot, you would need ROS code that subscribed to `/burlap_action` and turned the string represetnation into a set of physical actuations (either through additional ROS topic publishing or by being the driver of the robot). Often times, you will probably want a more direct connection to the Robot's actuators than the `ActionStringPublisher` provides. However, for illustrative purposes this example is convenient because to see that the BURLAP-ROS connection is working, we simply need to run the ROS command 

`rostopic echo burlap_action`

which will print the strings received from BURLAP as actions are executed. Note that in this example code we simply have a random  policy running, so you should see a random assortment of "action1," and "action2."

###Example 2
In the last example we setup an environment that published actions as string representations of the action name. This approach is only effective if you have some ROS code running that knows how to interpret the string representations
and actuate it on the robot, thereby requiring a middle man. A more direct way to control the robot is to have
action execution publish more typical ROS messages that specify the actuation. For example, on turtlebot robots, and various other robots, it is common to publish a ROS Twist message over a period of time to have the robot move. In this example code, we setup the BURLAP environment so that action execution results in publishing a Twist message for a fixed duration, thereby directly controlling the robot without a middle-man ROS script. We also then use an `EnvironmentShell` over the Environment we created so that you can manually control the robot through the terminal to test it out. 

```
public static void main(String [] args){

	//create a new domain with no state representation
	SADomain domain = new SADomain();

	//create action specification
	domain.addActionTypes(
		new UniversalActionType("forward"), 
		new UniversalActionType("backward"),
		new UniversalActionType("rotate")
		new UniversalActionType("rotate_ccw"));

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
	env.overrideFirstReceivedState(NullState.instance);


	//create a terminal controlled explorer to run on our environment
	//so that we can control the robot with the keyboard
	EnvironmentShell shell = new EnvironmentShell(domain, env);
	shell.start();

}
```
For this example, we don't care to maintain any state since we simply want to show you how to use the code to directly control a robot with Twist messages. Therefore, we create a BURLAP `SADomain` with some `ActionType` objects for a forward, backward, and rotate actions.

Next we create Twist objects that are part of the [java_rosbridge](https://github.com/h2r/java_rosbridge) library we are using. These objects are Java Beans that follow the same structure as the ROS Twist message: a linear `Vector3` component and an angular `Vector3` component. Note that if you were writing code for a different kind of robot that didn't respond to Twist messages and used a message type not in [java_rosbridge](https://github.com/h2r/java_rosbridge), you could simply create your own [Java Bean](https://en.wikipedia.org/wiki/JavaBeans) class for the ROS message and use that just as well.

For the ROS environment we have to provide standard information about the ROS Bridge URI and the state topic. We won't actually be using the state topic in this example, so we could give it any name.

After creating our `RosEnvironment` we set up a `ActionPublisher` for each of our BURLAP actions. Specifically, we use a `RepeatingActionPublisher`. A `RepeatingActionPublisher` will have the affect of publishing a specified message a fixed number of times at a specified period. Specifically, for each BURLAP action, we define a `RepeatingActionPublisher` that will publish the corresponding Twist message 5 times with a period of 500 milliseconds between each publish. We also have to specify the action topic for this message. We set it to the topic commonly used by the Turtlebot robot, but you should be sure set it to whichever topic your Twist-controlled robot will respond. Note that we set a synchronized flag for the  `RepeatingActionPublisher` publisher to true. This has the effect of the `ActionPublisher` blocking until it has published all 5 messages before returning control to the calling `Environment`. Because we set it to be synchronized, it will also automatically set the return delay to the period length, which will cause the `RosEnvironment` to wait an additional 500 millseconds after the `RepeatingActionPublisher` published its final 5th message thereby allowing time for that final message to have an affect.

Normally when interacting with a `RosEnvironment` no observations or actions will be permitted until it receives from ROS the first State message (indicating the initial state of the environment). In this example, however, we are not going to write any state generation code, so no message will ever arrive. Therefore, we use the `env.overrideFirstReceivedState(NullState.instance))` call to force the Environment to think the current state is the one provided (an empty `NullState`) so that we can continue without waiting for a message that will never come.

Finally, we use the standard BURLAP `EnvironmentShell` to allow us to manually interact with the `Environment` with the shell's ex command (use ex -h to see its help information in the shell)


### Custom State Messages

In the above examples, we talked about how to use `RosEnvironment` which subscribes to a ROS topic that is expected to publish BURLAP states via the message type `burlap_msgs/burlap_state`. However, in some cases you may want to subcribe to one or more different topics and build the BURLAP `State` from them in Java. If you wish to do that, then you will either want to extend `AbstractRosEnviroment` which includes the code for managing action publishing, but does not include any code for building the state (see its documentation for more information); or if the state is fully defined by one topic, you may wish to subclass `RosEnvironment` and override the method `unpackStateFromMsg(JsonNode data, String stringRep)` which is the method that handles parsing the JSON message from ROS into a BURLAP state (see its documentation for more information).

#### Large Message Sizes

If you're building your state from a custom message, it's possible the message you're receiving from ROS is very large, such as frames from a video feed. In these cases, it's likely that the message size is larger than what Jetty's websocket buffer size is by default. However, you can increase the buffer size by subclassing `RosBridge` and annotating the subclass to have a larger buffer size. You do not need to implement or override any methods; `RosBridge` is subclassed purely to give it a custom buffer size annotation. For example:

```
@WebSocket(maxTextMessageSize = 500 * 1024) public class BigMessageRosBridge extends RosBridge{}
```

If you then instantiate your subclass, connect with it, and use a AbstractRosEnvironment or RosEnvironment constructor that accepts a RosBridge instance (rather than one that asks for the URI), then your environment will be able to handle the larger message sizes. See the readme and Java doc for [java_rosbridge](https://github.com/h2r/java_rosbridge) for more information.


### RosShellCommand

Given the addition of an interactive shell to BURLAP, burlap_rosbridge also comes with an additional `ShellCommand` for interacting with RosBridge that you can add to your shells called `RosShellCommand`. This command allows you to echo a topic on ROS, publish to a topic, or a send a raw message to the RosBridge server. After adding it to your shell, see it's help with the -h option for more information.
