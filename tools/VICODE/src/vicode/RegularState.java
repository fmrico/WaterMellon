/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/



package vicode;

/**
 *
 * @author paco
 */
public class RegularState extends State {

    //private final static String IMAGE_PATH = BuilderGUI.WORKING_DIRECTORY + "/src/jmanager/figs/RegularState.png";
	
    public RegularState(String id) {
        super(RegularState.class.getResource("resources/RegularState.png"), id);
    }

	public void setGraphDebug(boolean debug) {
			
		if(!debug)
		{
			this.setTransparency(0.5f);
			this.repaint();
		}
		else{
			this.setTransparency(1.0f);	
			this.repaint();
		}
	}

}
