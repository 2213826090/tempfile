import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;

import javax.swing.DefaultListModel;
import javax.swing.JFrame;
import javax.swing.JList;
import javax.swing.JScrollPane;
import javax.swing.SwingUtilities;

public class ReorderListener extends MouseAdapter {

    private JList list;
    ArrayList<String> testCaseLeftList;
    private int pressIndex = 0;
    private int releaseIndex = 0;

    public ReorderListener(JList list, ArrayList<String> testCaseLeftList) {
        if (!(list.getModel() instanceof DefaultListModel)) {
            throw new IllegalArgumentException(
                    "List must have a DefaultListModel");
        }
        this.list = list;
        this.testCaseLeftList = testCaseLeftList;
    }

    @Override
    public void mousePressed(MouseEvent e) {
        pressIndex = list.locationToIndex(e.getPoint());
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        releaseIndex = list.locationToIndex(e.getPoint());
        if (releaseIndex != pressIndex && releaseIndex != -1) {
            reorder();
        }
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        mouseReleased(e);
        pressIndex = releaseIndex;
    }

    private void reorder() {
        DefaultListModel model = (DefaultListModel) list.getModel();
        Object dragee = model.elementAt(pressIndex);
        model.removeElementAt(pressIndex);
        model.insertElementAt(dragee, releaseIndex);
        String tmp;
        tmp = testCaseLeftList.get(pressIndex);
        testCaseLeftList.set(pressIndex, testCaseLeftList.get(releaseIndex));
        testCaseLeftList.set(releaseIndex, tmp);
    }
}