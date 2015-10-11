package missioncontrol;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.Arrays;
import javax.swing.JTextArea;


public class RosProcess{

	private Process p = null;
	
	public RosProcess(String cmdline) {
		
		cmdline = "xterm -hold -e "+ cmdline;
		ProcessBuilder pb = new ProcessBuilder(new ArrayList<String>(Arrays.asList(cmdline.split(" "))));
		
		try {
			p = pb.start();
		} catch (IOException e1) {
			
			e1.printStackTrace();
		}
	}

	public void destroy() {
		p.destroy();
	}

	boolean isRunning() {
		try {
			p.exitValue();
			return false;
		} catch (Exception e) {
			return true;
		}
	}
}
