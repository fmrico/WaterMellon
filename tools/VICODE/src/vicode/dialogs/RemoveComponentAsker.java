/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/



package vicode.dialogs;

import vicode.Component;

/**
 *
 * @author jesus
 */
public class RemoveComponentAsker extends RemoverAskerGeneric {

	private Component component2remove = null;
	
	public RemoveComponentAsker() {
		super();
	}
	
	protected void yesOptionClicked() {
		builderGUI.removeComponent(component2remove);

		//Después de borrar el estado, vuelve al modo normal.
		builderGUI.attemptChangeToNormalMode();
	}

	public void setRemoveComponentID(Component component) {
		component2remove = component;
		msg = "¿Esta seguro de que desea borrar el componente " + component.getId() + "?";
	}
}
