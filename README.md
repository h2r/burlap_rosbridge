burlap_rosbridge
================

A BURLAP library extension for creating a ROS BURLAP Environment where the ROS connection is handled via ROS Bridge.
Currently, only one class is provided: `AsynchronousRosEnvironment` (package `burlap.ros`), which is used for creating an Environment in which
the current state is received over ROSBridge and actions are published to a ROS topic (as a `std_msgs/String.msg` ROS message) over Ros Bridge. This environment is
an asynchronous environment, so there is no checking for action "completion." Instead, after each action execution,
the environment simply waits a specified time for the supplied action to complete and the current state to be updated
before returning control to the client code.

States are communicated to the BURLAP environment over ROS messages with type `burlap_msgs/burlap_state`. You can get
the neccessary ROS message definition from the [burlap_msgs](https://github.com/h2r/burlap_msgs) project.

If you need to do additional state processing not provided in the communicated ROS message (e.g., add additional "virutal" objects to the received state) you may do so by overriding the method `onStateReceive(State)`.

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
In the below code we assume that ROS is being run on the local host (port 9090 as default for ROS Bridge)
and there is a ROS topic named `/burlap_state` that has a Grid World state message being published.
Actions will be published to the topic `/burlap_action` and it is assumed some process on ROS is subscribe to that topic to actuate
them. We allow 2 seconds for action execution. The behavior of the robot is controlled via a random policy.

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

	//create environment with 2000ms (2s) action execution time
	AsynchronousRosEnvironment env = new AsynchronousRosEnvironment(domain, uri, stateTopic, actionTopic, 2000);
	env.blockUntilStateReceived();
	
	//optionally, uncomment the below so that you can see the received state printed to the terminal
	//env.setPrintStateAsReceived(true);

	//create a domain wrapper of the environment so that wrapped domain's actions go to
	//to the environment, rather than the normal GridWorld action simulator code.
	DomainEnvironmentWrapper envDomainWrapper = new DomainEnvironmentWrapper(domain, env);
	final Domain envDomain = envDomainWrapper.generateDomain();

	//create a random policy for control that connects to the environment wrapped domain
	Policy randPolicy = new Policy.RandomPolicy(envDomain);
	
	//begin behavior for 100 steps (200 seconds)
	randPolicy.evaluateBehavior(env.getCurState(), new NullRewardFunction(), 100);

}

```

