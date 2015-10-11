/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


package vicode;

import java.util.ArrayList;
import java.util.Iterator;

public class DebugFSM extends Thread
{
	private String id = null;
	private String ip = null;

	public DebugFSM(String _id, String _ip) {

		id = new String(_id);
		ip = new String(_ip);

	}

	public void run() {
		System.out.println("Running");
		int status = 0;
		Ice.Communicator ic = null;
		
		try {
			String[] args = {new String("")};
			ic = Ice.Util.initialize(args);
			
			Ice.ObjectPrx base = null;
			
			System.out.println(ip);
			
			if(!ip.equals(new String("127.0.0.1")))
				base = ic.stringToProxy("VicodeDebug:default -p 10000");
			else
				base = ic.stringToProxy("VicodeDebug:default -h "+ip+" -p 10000");
			
			bicacomms.VicodeDebugPrx vicodeDebug = bicacomms.VicodeDebugPrxHelper.checkedCast(base);

			if (vicodeDebug == null)
				throw new Error("Invalid proxy/Not conneccted");
			else
			{
				while(true) {
					int state = vicodeDebug.getState(id);
					
					BuilderGUI builderGUI  = BuilderGUI.getInstance();

					ArrayList<RegularState> states = builderGUI.getStates();
										
					Iterator<RegularState> it = states.iterator();
					while (it.hasNext()) {
						RegularState s = (RegularState) it.next();
						s.setGraphDebug(s.id.equals(states.get(state-1).id));
					}
			
					
					Thread.sleep(200);
				}
			}
		} catch (Ice.LocalException e) {
			e.printStackTrace();
			status = 1;
		} catch (Exception e) {
			System.err.println(e.getMessage());
			status = 1;
		}
		
		
		
	}
}

