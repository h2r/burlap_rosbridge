package burlap.ros;

import burlap.debugtools.MyTimer;
import burlap.shell.BurlapShell;
import burlap.shell.command.ShellCommand;
import com.fasterxml.jackson.databind.JsonNode;
import joptsimple.OptionParser;
import joptsimple.OptionSet;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * @author James MacGlashan.
 */
public class RosShellCommand implements ShellCommand {

	protected OptionParser parser = new OptionParser("rt:f:h*");
	protected RosBridge ros;

	public RosShellCommand(RosBridge ros) {
		this.ros = ros;
	}

	public String commandName() {
		return "ros";
	}

	public int call(BurlapShell shell, String argString, final Scanner is, final PrintStream os) {

		String [] parsed = this.parseWithQuotesAndEscapedQuotes(argString);
		OptionSet oset = this.parser.parse(parsed);
		List<String> args = (List<String>)oset.nonOptionArguments();

		if(oset.has("h")){

			os.println("ros pub topic msg_type msg\n" +
					   "ros send msg\n" +
					   "ros echo [-r][-t time_out][-f fragment_size] topic [msg_type]\n\n" +
					"For publishing, the msg is the JSON formatted string of the ROS message. You will probably need to enclose the message " +
					"in single quotes if there are spaces. Uses backslash for escape single quotes. " +
					"For example, ros pub my_topic std_msgs/String '{\"data\": \"hello world!\"}'.\n\n" +

					"send will send a raw message to the rosbridge server. You will need to make sure that the message format adheres " +
					"to the Rosbridge JSON message protocol. Like with publish, you may need to enclose the message with single quotes if there are spaces and use escape single quotes.\n\n" +

					"For echo, the message type is optional, but if it is not present and the topic does not already exist on ROS, echo will fail to receive data. " +
					"Rosbridge may also fail if you don't privde the message type and it's the first time you've published to it.\n " +
					"By default echo will only print the next message received, use -r to continually print every subsequent message. To regain " +
					"control of the shell when echo set to repeat, hit enter. If not repeating, " +
					"you can set a time out to have control return with the -t option. By default time out is set to 10 seconds.");

			return 0;
		}

		if(args.size() < 2){
			return -1;
		}

		if(args.get(0).equals("pub")){
			if(args.size() != 4){
				return -1;
			}

			ros.publishJsonMsg(args.get(1), args.get(2), args.get(3));


		}
		else if(args.get(0).equals("send")){
			if(args.size() != 2){
				return -1;
			}

			ros.sendRawMessage(args.get(1));
		}
		else if(args.get(0).equals("echo")){
			if(args.size() != 3 && args.size() != 2){
				return -1;
			}

			final String topic = args.get(1);
			String pmsgType = null;
			if(args.size() == 3){
				pmsgType = args.get(2);
			}
			final String msgType = pmsgType;
			final boolean repeat = oset.has("r");

			double ptimeout = 10.;
			if(oset.has("t")){
				ptimeout = Double.parseDouble((String)oset.valueOf("t"));
			}

			final double timeout = ptimeout;

			SubscriptionRequestMsg sub = SubscriptionRequestMsg.generate(topic)
					.setType(msgType)
					.setQueueLength(1)
					.setThrottleRate(1);

			if(oset.has("f")){
				int fragSize = Integer.parseInt((String)oset.valueOf("f"));
				sub.setFragmentSize(fragSize);
			}

			if(!repeat){

				final SingleEchoer echoer = new SingleEchoer(os, this);



				ros.subscribe(sub, echoer);


				final Thread echoThread = new Thread(new Runnable() {

					public void run() {
						MyTimer time = new MyTimer(true);
						synchronized(RosShellCommand.this){
							while(echoer.done == false && time.peekAtTime() < timeout){
								try {
									RosShellCommand.this.wait((long)(timeout*1000));
								} catch(InterruptedException e) {
									e.printStackTrace();
								}
							}
						}
						time.stop();
						if(time.getTime() > timeout){
							os.println("Timed out... Consider providing ros message type if you have not subscribed to this topic before.");
						}
						ros.removeListener(topic, echoer);
					}
				});

				echoThread.start();
				try {
					echoThread.join();
				} catch(InterruptedException e) {
					e.printStackTrace();
				}

			}
			else{

				RosListenDelegate echoer = new RosListenDelegate() {

					public void receive(JsonNode data, String stringRep) {
						JsonNode rosMsgNode = data.get("msg");
						String msgFormat = rosMsgNode.toString();
						os.println(msgFormat);
					}
				};

				ros.subscribe(sub, echoer);
				is.nextLine(); //block until input
				ros.removeListener(topic, echoer);


			}
		}

		return 0;
	}


	protected String[] parseWithQuotesAndEscapedQuotes(String input){

		List<String> list = new ArrayList<String>();
		Matcher m = Pattern.compile("([^']\\S*|'.+?(?<!\\\\)')\\s*").matcher(input);
		while (m.find()) {
			String match = m.group(1).replace("\\'", "'");
			if(match.startsWith("'") && match.endsWith("'")){
				match = match.substring(1, match.length()-1);
			}
			list.add(match); // Add .replace("\"", "") to remove surrounding quotes.
		}

		String [] array = list.toArray(new String[list.size()]);

		return array;

	}


	protected class SingleEchoer implements RosListenDelegate{

		PrintStream os;
		public boolean done = false;
		Object lock;

		public SingleEchoer(PrintStream os, Object lock) {
			this.os = os;
			this.lock = lock;
		}


		public void receive(JsonNode data, String stringRep) {
			JsonNode rosMsgNode = data.get("msg");
			String msgFormat = rosMsgNode.toString();
			os.println(msgFormat);
			done = true;
			synchronized(lock){
				lock.notifyAll();
			}
		}
	}


}
