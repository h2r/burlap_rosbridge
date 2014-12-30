burlap_rosbridge
================

A library for creating a ROS BURLAP Environment, where the ROS connection is handled via ROS Bridge.
Currently, only one class is provided AsynchronousRosEnvironment, which is used for creating an Environment in which
the current state is received over ROSBridge and actions are published to a topic (as a string) to Ros Bridge. This is
an asynchronous environment, so there is no checking for action "completion." Instead, after each action execution,
the environment simply waits a specified time for the supplied action to complete and the current state to be updated
before returning control to the client code. More information below.

States are communicated to the BURLAP environment over ROS messages adhering to the burlap_msgs/burlap_state ROS message type.

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

	//create environment
	RosEnvironment env = new RosEnvironment(domain, uri, stateTopic, actionTopic, 2000);
	env.blockUntilStateReceived();

	//create a domain wrapper of the environment to handle policy execution
	DomainEnvironmentWrapper envDomainWrapper = new DomainEnvironmentWrapper(domain, env);
	final Domain envDomain = envDomainWrapper.generateDomain();

	//create a random policy for control
	Policy randPolicy = new Policy.RandomPolicy(envDomain);
		
	//begin behavior for 100 steps (200 seconds)
	randPolicy.evaluateBehavior(env.getCurState(), new NullRewardFunction(), 100);

}

```

