/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


package launcher;

import java.awt.EventQueue;

import javax.swing.JFrame;

import java.awt.GridBagLayout;

import javax.swing.JTextField;

import java.awt.GridBagConstraints;

import javax.swing.JList;

import java.awt.Color;
import java.awt.Insets;
import java.awt.FlowLayout;

import javax.swing.JInternalFrame;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JCheckBox;

import bicacomms.ComponentsList;
import vicode.BuilderGUI;
import vicode.RegularState;

import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

public class Launcher {

	private JFrame frame;
	private JTextField textField;
	JComboBox ComponentsBox;

	private Ice.Communicator ic = null;
	private bicacomms.VicodeDebugPrx vicodeDebug = null;
	private bicacomms.SchedInterfacePrx schedI = null;
	private JButton btnRemove = null;
	private JButton btnAdd = null;
	private JCheckBox chckbxMultipleActivations = null;

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					Launcher window = new Launcher();
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
	public Launcher() {
		initialize();
	}

	/**
	 * Initialize the contents of the frame.
	 */
	private void initialize() {
		frame = new JFrame();
		frame.setBounds(100, 100, 378, 474);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.getContentPane().setLayout(null);

		JButton ConnectButton = new JButton("Connect");
		ConnectButton.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseClicked(MouseEvent e) {


				int status = 0;

				try {
					String[] args = {new String("")};
					ic = Ice.Util.initialize(args);
					Ice.ObjectPrx base = null;		
					String ip = textField.getText();

					if(!ip.equals(new String("127.0.0.1")))
						base = ic.stringToProxy("VicodeDebug:default -p 10000");
					else
						base = ic.stringToProxy("VicodeDebug:default -h "+ip+" -p 10000");

					vicodeDebug = bicacomms.VicodeDebugPrxHelper.checkedCast(base);

					if (vicodeDebug == null)
						throw new Error("Invalid proxy/Not conneccted");
					else
					{
						ComponentsList compos = vicodeDebug.getListComponents();
						System.out.println("Num: "+compos.numCompos);

						ComponentsBox.removeAllItems();
						
						for(int i=0; i<compos.numCompos;i++)
						{
							System.out.println("["+compos.ListComps[i]+"]");
							ComponentsBox.addItem(compos.ListComps[i]);						
						}

						btnRemove.setEnabled(true);
						btnAdd.setEnabled(true);

					}
				} catch (Ice.LocalException e1) {
					e1.printStackTrace();
					status = 1;
				} catch (Exception e1) {
					System.err.println(e1.getMessage());
					status = 1;
				}


			}
		});
		ConnectButton.setBounds(216, 12, 117, 25);
		frame.getContentPane().add(ConnectButton);

		textField = new JTextField();
		textField.setText("127.0.0.1");
		textField.setBounds(41, 15, 114, 19);
		frame.getContentPane().add(textField);
		textField.setColumns(10);

		ComponentsBox = new JComboBox();
		ComponentsBox.setBounds(41, 82, 292, 24);
		frame.getContentPane().add(ComponentsBox);

		btnAdd = new JButton("Add");
		btnAdd.setEnabled(false);
		btnAdd.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseClicked(MouseEvent e) {
				String selected = (String) ComponentsBox.getItemAt(ComponentsBox.getSelectedIndex());
				System.out.println("["+selected+"]");

				if(schedI==null)
				{
					Ice.ObjectPrx base = null;		

					String ip = textField.getText();


					if(!ip.equals(new String("127.0.0.1")))
						base = ic.stringToProxy("SchedInterface:default -p 10000");
					else
						base = ic.stringToProxy("SchedInterface:default -h "+ip+" -p 10000");

					if(base!=null)
						schedI = bicacomms.SchedInterfacePrxHelper.checkedCast(base);
				}
				
				if (schedI == null)
					throw new Error("Invalid proxy/Not conneccted");
				else
				{
					if(!chckbxMultipleActivations.isSelected())
						schedI.removeAll();
					int ret  = schedI.addSched(selected); //Is the returned value useful?
					
					//ComponentsBox.getComponent(ComponentsBox.getSelectedIndex()).setForeground(Color.GREEN); 

					//if((ret == bicacomms.SUCCESS.value))
					//	ComponentsBox.getComponent(ComponentsBox.getSelectedIndex()).setForeground(Color.GREEN); 
				}
			}
		});

		btnAdd.setBounds(41, 156, 117, 25);
		frame.getContentPane().add(btnAdd);

		btnRemove = new JButton("Remove");
		btnRemove.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseClicked(MouseEvent e) {
				String selected = (String) ComponentsBox.getItemAt(ComponentsBox.getSelectedIndex());
				System.out.println("["+selected+"]");

				if(schedI==null)
				{
					Ice.ObjectPrx base = null;		

					String ip = textField.getText();


					if(!ip.equals(new String("127.0.0.1")))
						base = ic.stringToProxy("SchedInterface:default -p 10000");
					else
						base = ic.stringToProxy("SchedInterface:default -h "+ip+" -p 10000");

					if(base!=null)
						schedI = bicacomms.SchedInterfacePrxHelper.checkedCast(base);
				}
				
				if (schedI == null)
					throw new Error("Invalid proxy/Not conneccted");
				else
				{
					if(!chckbxMultipleActivations.isSelected())
						schedI.removeAll();
					
					int ret = schedI.removeSched(selected); //Is the returned value useful?

					//ComponentsBox.getComponent(ComponentsBox.getSelectedIndex()).setForeground(Color.BLACK); 
					
					//if((ret == bicacomms.SUCCESS.value) || (ret == bicacomms.NOTRUNNING.value))
					//	ComponentsBox.getComponent(0).setForeground(Color.BLACK); 
				}
			}
		});
		btnRemove.setEnabled(false);
		btnRemove.setBounds(216, 156, 117, 25);
		frame.getContentPane().add(btnRemove);

		chckbxMultipleActivations = new JCheckBox("Multiple activations");
		chckbxMultipleActivations.setBounds(89, 245, 200, 23);
		frame.getContentPane().add(chckbxMultipleActivations);
	}
}
