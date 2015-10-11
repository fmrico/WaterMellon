package missioncontrol;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.EventQueue;

import javax.swing.ButtonGroup;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JToggleButton;
import javax.swing.ScrollPaneLayout;

import java.awt.Color;
import javax.swing.JTextField;
import javax.swing.JTabbedPane;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import javax.swing.JRadioButtonMenuItem;
import javax.swing.JMenu;
import javax.swing.JLayeredPane;
import javax.swing.border.TitledBorder;
import javax.swing.JButton;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import javax.swing.JCheckBox;
import java.awt.event.ItemListener;
import java.awt.event.ItemEvent;
import java.util.Date;

public class MissionControl {

	private JFrame frmMissioncontrol;
	private JTextField RoscoreHost;
	private JTextField RoscorePort;
	private JToggleButton tglbtnRoscore;
	private JToggleButton tglbtnBica;

	private JTabbedPane tabbedPane;
	private JRadioButtonMenuItem rdbtnmntmReal = null;
	JTextArea textarea = null;

	private RosProcessManager rosProcesses = null;
	private JLayeredPane layeredPane_1;

	private long initTime;

	private Ice.Communicator ic = null;
	private bicacomms.SchedInterfacePrx schedI = null;
	private JLabel lblTestGlobalnavigator;
	private JToggleButton tglbtnTestGlobalNavigator;

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					MissionControl window = new MissionControl();
					window.frmMissioncontrol.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the application.
	 */
	public MissionControl() {
		initialize();

		rosProcesses = new RosProcessManager();
		rosProcesses.start();
		Date date = new Date();
		initTime = date.getTime();

	}

	/**
	 * Initialize the contents of the frame.
	 */
	private void initialize() {

		frmMissioncontrol = new JFrame();
		frmMissioncontrol.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				System.err.println("Cleanning");
				rosProcesses.destroyAll();
			}
		});
		frmMissioncontrol.setTitle("MissionControl");
		frmMissioncontrol.setBounds(100, 100, 1024, 768);
		frmMissioncontrol.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frmMissioncontrol.getContentPane().setLayout(null);

		JLayeredPane layeredPane = new JLayeredPane();
		layeredPane.setBorder(new TitledBorder(null, "Robot",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		layeredPane.setBounds(810, 28, 155, 77);
		frmMissioncontrol.getContentPane().add(layeredPane);

		rdbtnmntmReal = new JRadioButtonMenuItem("Real");
		rdbtnmntmReal.setBounds(12, 24, 87, 19);
		layeredPane.add(rdbtnmntmReal);

		final JRadioButtonMenuItem rdbtnmntmSimulated = new JRadioButtonMenuItem(
				"Simulated");
		rdbtnmntmSimulated.setSelected(true);
		rdbtnmntmSimulated.setBounds(12, 44, 118, 19);
		layeredPane.add(rdbtnmntmSimulated);

		final ButtonGroup bg = new ButtonGroup();
		bg.add(rdbtnmntmReal);
		bg.add(rdbtnmntmSimulated);

		JLabel lblRoscore = new JLabel("Robot");
		lblRoscore.setBounds(31, 28, 70, 15);
		frmMissioncontrol.getContentPane().add(lblRoscore);

		tglbtnRoscore = new JToggleButton("Start");

		tglbtnRoscore.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseReleased(MouseEvent e) {
				if (tglbtnRoscore.isSelected())
					startRobot();
				else
					stopRobot();
			}
		});
		tglbtnRoscore.setBackground(Color.RED);
		tglbtnRoscore.setBounds(106, 23, 103, 25);
		frmMissioncontrol.getContentPane().add(tglbtnRoscore);

		RoscoreHost = new JTextField();
		RoscoreHost.setText("127.0.0.1");
		RoscoreHost.setBounds(232, 26, 136, 19);
		frmMissioncontrol.getContentPane().add(RoscoreHost);
		RoscoreHost.setColumns(10);

		RoscorePort = new JTextField();
		RoscorePort.setText("11311");
		RoscorePort.setBounds(380, 26, 78, 19);
		frmMissioncontrol.getContentPane().add(RoscorePort);
		RoscorePort.setColumns(10);

		layeredPane_1 = new JLayeredPane();
		layeredPane_1.setBorder(new TitledBorder(null, "Console",
				TitledBorder.LEADING, TitledBorder.TOP, null, null));
		layeredPane_1.setBounds(12, 441, 998, 286);
		frmMissioncontrol.getContentPane().add(layeredPane_1);

		tabbedPane = new JTabbedPane(JTabbedPane.TOP);
		tabbedPane.setBounds(0, 106, 985, 168);
		layeredPane_1.add(tabbedPane);

		textarea = new JTextArea();
		JScrollPane pane = new JScrollPane(textarea);

		pane.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
		pane.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);

		tabbedPane.add("Console", pane);

		final JCheckBox chckbxRosout = new JCheckBox("rosout");
		chckbxRosout.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent arg0) {
				if (chckbxRosout.isSelected())
					startRosout();
				else
					stopRosout();
			}
		});

		chckbxRosout.setBounds(8, 30, 129, 23);
		layeredPane_1.add(chckbxRosout);

		JLabel lblBica = new JLabel("Bica");
		lblBica.setBounds(31, 90, 70, 15);
		frmMissioncontrol.getContentPane().add(lblBica);

		tglbtnBica = new JToggleButton("Start");
		tglbtnBica.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseReleased(MouseEvent e) {
				if (tglbtnBica.isSelected())
					startBica();
				else
					stopBica();
			}
		});
		tglbtnBica.setBackground(Color.RED);
		tglbtnBica.setBounds(106, 85, 103, 25);
		frmMissioncontrol.getContentPane().add(tglbtnBica);
		
		lblTestGlobalnavigator = new JLabel("Test GlobalNavigator");
		lblTestGlobalnavigator.setBounds(31, 159, 155, 15);
		frmMissioncontrol.getContentPane().add(lblTestGlobalnavigator);
		
		tglbtnTestGlobalNavigator = new JToggleButton("Start");
		tglbtnTestGlobalNavigator.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseReleased(MouseEvent e) {
				if (tglbtnTestGlobalNavigator.isSelected())
					startTestGlobalNavigator();
				else
					stopTestGlobalNavigator();
			}
		});
		tglbtnTestGlobalNavigator.setBackground(Color.RED);
		tglbtnTestGlobalNavigator.setBounds(218, 149, 103, 25);
		frmMissioncontrol.getContentPane().add(tglbtnTestGlobalNavigator);

	}

	protected void stopTestGlobalNavigator() {
		
		
		if(schedI.removeSched("TestGlobalNavigator")==0)
		{
			tglbtnRoscore.setBackground(Color.RED);
			tglbtnRoscore.setText("Start");
			rosProcesses.stopProcess("navigation");
			log("TestGlobalNavigator stopped \n");
		}else
		{
			log("TestGlobalNavigator FAILED to stop \n");
		}
		
	}

	protected void startTestGlobalNavigator() {
		if(!rosProcesses.isRunning("bica")){
			startBica();
		}
		
		String cmd = new String();
		
		if(schedI.addSched("TestGlobalNavigator")==0)
		{
			log("TestGlobalNavigator started \n");
			
			cmd = "roslaunch cervantes_navigation usar_mapa.launch";
			if (rosProcesses.startProcess("navigation", cmd)) 
				log("Robot started \n\t[" + cmd + "]\n");
				
			tglbtnTestGlobalNavigator.setBackground(Color.GREEN);
			tglbtnTestGlobalNavigator.setText("Stop");
		}else
		{
			log("TestGlobalNavigator FAILED to start \n");
			tglbtnTestGlobalNavigator.setSelected(false);
		}
		
	}

	protected void startRosout() {
		log("rosout started\n");
		rosProcesses.startProcess("rosout", "rostopic echo /rosout");
	}

	protected void stopRosout() {
		log("rosout stopped\n");
		rosProcesses.stopProcess("rosout");
	}

	protected void startRobot() {
		String cmd = new String();

		if (rdbtnmntmReal.isSelected())
			cmd = "roscore -p " + RoscorePort.getText();
		else
			cmd = "roslaunch cervantes_robot_model cervantes_gazebo.launch";

		if (rosProcesses.startProcess("robot", cmd)) {

			log("Robot started \n\t[" + cmd + "]\n");

			tglbtnRoscore.setBackground(Color.GREEN);
			tglbtnRoscore.setText("Stop");
		} else {
			log("robot FAILED to start \n");
			tglbtnRoscore.setSelected(false);
		}

	}

	protected void stopRobot() {
		rosProcesses.stopProcess("robot");
		tglbtnRoscore.setBackground(Color.RED);
		tglbtnRoscore.setText("Start");

		log("Robot stopped \n");
	}

	protected void startBica() {
		
		if(!rosProcesses.isRunning("robot")){
			startRobot();
		}
		
		String cmd = new String();

		cmd = "rosrun bica bica";

		if (rosProcesses.startProcess("bica", cmd)) {

			log("Bica started \n\t[" + cmd + "]\n");

			tglbtnBica.setBackground(Color.GREEN);
			tglbtnBica.setText("Stop");

			while(!startICEComms());

		} else {
			log("Bica FAILED to start \n");
			tglbtnBica.setSelected(false);
		}

	}

	private boolean startICEComms() {

		try {
			String[] args = { new String("") };
			ic = Ice.Util.initialize(args);
			Ice.ObjectPrx base = null;

			String ip = RoscoreHost.getText();

			if (!ip.equals(new String("127.0.0.1")))
				base = ic.stringToProxy("SchedInterface:default -p 10000");
			else
				base = ic.stringToProxy("SchedInterface:default -h " + ip
						+ " -p 10000");

			if (base != null)
				schedI = bicacomms.SchedInterfacePrxHelper.checkedCast(base);

			log("ICE comms up&running \n");
			return true;
		} catch (Ice.ConnectionRefusedException e) {
			log("ICE not ready... \n");
			return false;
		}

	}

	protected void stopBica() {
		rosProcesses.stopProcess("bica");
		tglbtnBica.setBackground(Color.RED);
		tglbtnBica.setText("Start");

		log("Bica stopped \n");
	}

	private void log(String msg) {
		long elapsed;
		Date date = new Date();
		elapsed = date.getTime() - initTime;

		textarea.append("[" + Long.toString(elapsed) + "]\t" + msg);

	}
}
