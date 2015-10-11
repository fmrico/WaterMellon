/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


package vicode.dialogs;

import javax.swing.JOptionPane;
import vicode.BuilderGUI;

/**
 *
 * @author jesus
 */
public abstract class RemoverAskerGeneric {

	protected String msg;
	protected BuilderGUI builderGUI;

	protected RemoverAskerGeneric() {
		builderGUI = BuilderGUI.getInstance();
		msg = "";
	}

	protected abstract void yesOptionClicked();

	public void showDialog() {
		int selection = JOptionPane.showOptionDialog(null, msg, "Aviso", JOptionPane.YES_NO_OPTION,
			JOptionPane.INFORMATION_MESSAGE, null, new Object[]{"YES", "NO"}, "NO");

		if (selection == 0) { //Se eligió "YES"
			this.yesOptionClicked();

		} else {
			//si no está pulsado el botón de 'hold', vuelve al modo normal
			//y desactiva cualquier botón que hubiera.
			builderGUI.attemptChangeToNormalMode();
		}
	}
}
