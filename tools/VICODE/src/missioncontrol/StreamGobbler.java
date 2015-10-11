package missioncontrol;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

import javax.swing.JTextArea;


class StreamGobbler extends Thread {
    InputStream is;
    JTextArea text;

    StreamGobbler(InputStream is, JTextArea text) {
        this.is = is;
        this.text = text;
    }

    @Override
    public void run() {
        InputStreamReader isr = new InputStreamReader(is);
		BufferedReader br = new BufferedReader(isr, 1);
		
		String line = null;
		try{
			while( (line = br.readLine()) != null){
				text.append(line+"\n");
			}
				
		}catch(java.io.IOException e){
			System.out.println("Stream cerrado");
		}
    }
}