package missioncontrol;
import java.awt.List;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

public class RosProcessManager extends Thread{

	private Map<String, RosProcess> processList = new HashMap<String, RosProcess>();
	private boolean finish=false;
	
	public RosProcessManager() {
		
	}

	public boolean startProcess(String id, String cmdline) {

		processList.put(id, new RosProcess(cmdline) );

		return true;
	}

	public void stopProcess(String id) {

		if (processList.containsKey(id)) {
			processList.get(id).destroy();
			processList.remove(id);
		}

	}
	
	public boolean isRunning(String id) {

		if (processList.containsKey(id) && processList.get(id).isRunning())
			return true;
		else
			return false;

	}
	public void destroyAll() {

		Iterator<Entry<String, RosProcess>> it = processList.entrySet().iterator();		
		while (it.hasNext()) {
			Entry<String, RosProcess> pairs = it.next();
			stopProcess((String) pairs.getKey());
		}
	}
	
	public void run() {
		finish = true;
		while(!finish){
			
		}
	
    }	

}
