/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


package vicode;

import java.awt.EventQueue;

import javax.swing.JFrame;
import javax.swing.JPanel;

import java.awt.BorderLayout;

import javax.swing.JCheckBox;
import javax.swing.JFormattedTextField;
import javax.swing.JSplitPane;
import javax.swing.JTextField;

import java.awt.Panel;
import java.awt.FlowLayout;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.event.ItemListener;
import java.awt.event.ItemEvent;
import java.awt.Label;
import java.util.ArrayList;
import java.util.Iterator;

public class Vicode {

	private JFrame frame;
	private JTextField textField;
	private DebugFSM debugFSM = null;
	private Label label = null;
	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {		
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					Vicode window = new Vicode();
					window.frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the application.
	 */
	public Vicode() {
		initialize();
	}

	/**
	 * Initialize the contents of the frame.
	 */
	private void initialize() {
		frame = new JFrame();
		frame.setBounds(100, 100, 1200, 640);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		Builder panel = new Builder();
		frame.getContentPane().add(panel, BorderLayout.CENTER);
		
		Panel panel_1 = new Panel();
		frame.getContentPane().add(panel_1, BorderLayout.SOUTH);
		panel_1.setLayout(new FlowLayout(FlowLayout.CENTER, 5, 5));
	
		
		final JCheckBox debugCheckBox = new JCheckBox("Debug");
		debugCheckBox.addItemListener(new ItemListener() {
			@SuppressWarnings("deprecation")
			public void itemStateChanged(ItemEvent arg0) {
				if(debugCheckBox.isSelected())
				{
					String id = Builder.getInstance().getId();
					
					if(id!=null) {
						label.setText("");
						
						debugFSM = new DebugFSM(id, textField.getText());
						debugFSM.start();
					}else
					{
						label.setSize(300, 20);
						label.setText("Component ID not set");
					}
				}
				else {
					
					BuilderGUI builderGUI  = BuilderGUI.getInstance();

					ArrayList<RegularState> states = builderGUI.getStates();
										
					Iterator<RegularState> it = states.iterator();
					while (it.hasNext()) {
						RegularState s = (RegularState) it.next();
						s.setGraphDebug(true);
					}
			
					
					label.setText("");
					if(debugFSM != null) {
						debugFSM.stop();
						debugFSM = null;
					}
				
				}
					
			}
		});
		
		panel_1.add(debugCheckBox);
		
		textField = new JTextField();
		textField.setText("127.0.0.1");
		panel_1.add(textField);
		textField.setColumns(10);
		
		label = new Label("");
		panel_1.add(label);


	}

}
