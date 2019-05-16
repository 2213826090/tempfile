import javax.swing.*;
import javax.swing.event.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

//import java.nio.file.Files;

public class ACSTCInterface extends JPanel {
    JTextArea swingConsoleOutput;
    JList swingLeftList;
    JList swingRightList;
    ListSelectionModel listSelectionModel;
    ListSelectionModel listSelectionModel2;
    JScrollPane swingLeftListPane;
    JScrollPane swingRightListPane;
    DefaultListModel listModel;
    DefaultListModel listModel2;
    JButton swingAddButton;
    JButton swingRemoveButton;
    JPanel swingParameterLabel;
    JPanel swingParameterValue;
    ScrollPane swingParameterPane;
    JPanel swingSave;
    JTextField swingSearchBox;

    ListSelectionModel lsm;

    int selectedItemLeftList = 0;
    int selectedItemRightList = 0;

    CTGenerator CTG;
    GridBagConstraints gBC;

    ArrayList<String> initialTCList = new ArrayList<String>();
    ArrayList<String> testCaseLeftList = new ArrayList<String>();
    ArrayList<String> testCaseRightList = new ArrayList<String>();
    ArrayList<JTextField> buttonsGeneratedList = new ArrayList<JTextField>();

    String newline = "\n";
    String outputTCGPath = "../";

    public ACSTCInterface() {

        super(new GridBagLayout());
        CTG = new CTGenerator();
        gBC = new GridBagConstraints();

        // LeftList
        listModel = new DefaultListModel();
        swingLeftList = new JList(listModel);
        swingLeftListPane = new JScrollPane(swingLeftList);
        listSelectionModel = swingLeftList.getSelectionModel();
        listSelectionModel.addListSelectionListener(new ListSelectionHandler());
        swingLeftListPane.setPreferredSize(new Dimension(400, 300));
        swingLeftListPane.setMinimumSize(new Dimension(400, 300));
        gBC.fill = GridBagConstraints.BOTH;
        gBC.weightx = 1;
        gBC.weighty = 1;
        gBC.gridx = 0;
        gBC.gridy = 0;
        gBC.insets = new Insets(10, 10, 10, 10);

        MouseAdapter dragListener = new ReorderListener(swingLeftList,
                testCaseLeftList);
        swingLeftList.addMouseListener(dragListener);
        swingLeftList.addMouseMotionListener(dragListener);

        add(swingLeftListPane, gBC);

        // Add/Remove Buttons
        swingAddButton = new JButton("add");
        swingAddButton.addActionListener(new ButtonAddHandler());
        swingAddButton.setMinimumSize(new Dimension(200, 30));
        swingAddButton.setPreferredSize(new Dimension(200, 30));
        gBC.fill = GridBagConstraints.NONE;
        gBC.weightx = 0.5;
        gBC.weighty = 0.5;
        gBC.gridx = 1;
        gBC.gridy = 0;
        gBC.insets = new Insets(10, 10, 10, 10);
        add(swingAddButton, gBC);

        swingRemoveButton = new JButton("remove");
        swingRemoveButton.addActionListener(new ButtonRemoveHandler());
        swingRemoveButton.setMinimumSize(new Dimension(200, 30));
        swingRemoveButton.setPreferredSize(new Dimension(200, 30));
        gBC.fill = GridBagConstraints.NONE;
        gBC.weightx = 0.5;
        gBC.weighty = 0.5;
        gBC.gridx = 2;
        gBC.gridy = 0;
        gBC.insets = new Insets(10, 10, 10, 10);
        add(swingRemoveButton, gBC);

        // RightList
        testCaseRightList = CTG.importTCList();

        listModel2 = new DefaultListModel();
        swingRightList = new JList(listModel2);
        swingRightListPane = new JScrollPane(swingRightList);
        listSelectionModel2 = swingRightList.getSelectionModel();
        listSelectionModel2
                .addListSelectionListener(new ListSelectionHandler());
        for (int i = 0; i < testCaseRightList.size(); i++) {
            listModel2.addElement(testCaseRightList.get(i));
        }
        swingRightListPane.setPreferredSize(new Dimension(400, 300));
        swingRightListPane.setMinimumSize(new Dimension(400, 300));
        gBC.fill = GridBagConstraints.BOTH;
        gBC.weightx = 1;
        gBC.gridx = 3;
        gBC.gridy = 0;
        gBC.insets = new Insets(10, 10, 10, 10);
        add(swingRightListPane, gBC);

        // Search box

        swingSearchBox = new JTextField();
        swingSearchBox.setMinimumSize(new Dimension(50, 100));
        swingSearchBox.addKeyListener(new KeyListener() {

            @Override
            public void keyTyped(KeyEvent arg0) {
                // TODO Auto-generated method stub

            }

            @Override
            public void keyReleased(KeyEvent arg0) {
            }

            @Override
            public void keyPressed(KeyEvent arg0) {

                if (arg0.getKeyCode() == KeyEvent.VK_ENTER) {
                    String searchText = swingSearchBox.getText().toLowerCase();
                    if (!searchText.equals("")) {
                        DefaultListModel listModelSearch = new DefaultListModel();
                        listModel2.clear();
                        for (int i = 0; i < testCaseRightList.size(); i++) {
                            if (testCaseRightList.get(i).toLowerCase()
                                    .contains(searchText)) {
                                listModel2.addElement((testCaseRightList.get(i)));
                            }
                        }
                    } else {
                        DefaultListModel listModelSearch = new DefaultListModel();
                        listModel2.clear();
                        for (int i = 0; i < testCaseRightList.size(); i++) {
                            listModel2.addElement(testCaseRightList.get(i));
                        }
                    }
                }
            }
        });
        gBC.weightx = 1;
        gBC.gridx = 3;
        gBC.gridy = 1;
        gBC.gridwidth = 1;
        add(swingSearchBox, gBC);

        // Console - disabled

        swingConsoleOutput = new JTextArea(1, 10);
        swingConsoleOutput.setEditable(false);
        swingConsoleOutput.setMinimumSize(new Dimension(300, 300));
        JScrollPane outputPane = new JScrollPane(swingConsoleOutput,
                ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS,
                ScrollPaneConstants.HORIZONTAL_SCROLLBAR_AS_NEEDED);
        gBC.fill = GridBagConstraints.BOTH;
        gBC.weightx = 1;
        gBC.gridx = 0;
        gBC.gridy = 1;
        gBC.gridwidth = 4;
        // add(outputPane, gBC);

        // TCParameters
        JPanel mid = new JPanel(new GridBagLayout());
        // mid.setPreferredSize(new Dimension(200, 300));
        swingParameterPane = new ScrollPane();
        swingParameterPane.setPreferredSize(new Dimension(200, 300));
        gBC.fill = GridBagConstraints.BOTH;
        gBC.weightx = 1;
        gBC.weighty = 1;
        gBC.gridx = 0;
        gBC.gridy = 2;
        gBC.gridwidth = 2;
        add(swingParameterPane, gBC);
        swingParameterLabel = new JPanel();
        swingParameterLabel.setLayout(new BoxLayout(swingParameterLabel,
                BoxLayout.Y_AXIS));
        gBC.fill = GridBagConstraints.BOTH;
        gBC.weightx = 1;
        gBC.weighty = 1;
        gBC.gridx = 0;
        gBC.gridy = 2;
        gBC.gridwidth = 1;
        mid.add(swingParameterLabel, gBC);

        swingParameterValue = new JPanel();
        swingParameterValue.setLayout(new BoxLayout(swingParameterValue,
                BoxLayout.Y_AXIS));
        gBC.fill = GridBagConstraints.BOTH;
        gBC.weightx = 1;
        gBC.weighty = 1;
        gBC.gridx = 1;
        gBC.gridy = 2;
        gBC.gridwidth = 1;
        mid.add(swingParameterValue, gBC);
        swingParameterPane.add(mid, gBC);

        swingSave = new JPanel();
        mid.add(swingSave);

        // gBC.fill = GridBagConstraints.BOTH;
        // gBC.weightx = 1;
        // gBC.weighty = 1;
        // gBC.gridx = 0;
        // gBC.gridy = 2;
        // gBC.gridwidth = 5;
        mid.add(swingParameterValue, gBC);

        // Generate button
        JButton generateButton = new JButton("generate");
        generateButton.setMinimumSize(new Dimension(200, 30));
        swingParameterPane.setPreferredSize(new Dimension(200, 300));
        generateButton.addActionListener(new ButtonGenerateCampaignHandler());
        gBC.fill = GridBagConstraints.NONE;
        gBC.weightx = 1;
        gBC.weighty = 1;
        gBC.gridx = 0;
        gBC.gridy = 3;
        gBC.gridwidth = 1;
        gBC.insets = new Insets(10, 10, 10, 10);
        add(generateButton, gBC);

        // In progress
        /*
         * JButton runACS = new JButton("runACS"); runACS.setMinimumSize(new
         * Dimension(200, 30)); runACS.addActionListener(new ActionListener() {
         * 
         * @Override public void actionPerformed(ActionEvent arg0) { try {
         * String[] cmd = new String[]{"/bin/sh",
         * "/home/razvan/Programe/acs/acs/src/_ExecutionConfig/startACS.sh"};
         * Process pr = Runtime.getRuntime().exec(cmd); pr.waitFor();
         * System.out.println(""+pr.exitValue());
         * 
         * int len; if ((len = pr.getErrorStream().available()) > 0) { byte[]
         * buf = new byte[len]; pr.getErrorStream().read(buf);
         * System.err.println("Command error:\t\""+new String(buf)+"\""); }
         * 
         * 
         * } catch (IOException e) {
         * System.out.println(""+e.getLocalizedMessage()); e.printStackTrace();
         * } catch (InterruptedException e) {
         * System.out.println(""+e.getLocalizedMessage());
         * 
         * // TODO Auto-generated catch block e.printStackTrace(); } } });
         * gBC.fill = GridBagConstraints.NONE; gBC.weightx = 1; gBC.weighty = 1;
         * gBC.gridx = 3; gBC.gridy = 3; gBC.gridwidth = 1; add(runACS, gBC);
         */
        // setPreferredSize(new Dimension(1200, 1000));
        setPreferredSize(getPreferredSize());
    }

    /**
     * Create the GUI and show it. For thread safety, this method should be
     * invoked from the event-dispatching thread.
     */
    private static void createAndShowGUI() {
        // Create and set up the window.
        JFrame frame = new JFrame("ListSelectionDemo");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        // Create and set up the content pane.
        ACSTCInterface demo = new ACSTCInterface();
        demo.setOpaque(true);
        frame.setContentPane(demo);

        // Display the window.
        frame.pack();
        frame.setVisible(true);
    }

    public static void main(String[] args) {
        // Schedule a job for the event-dispatching thread:
        // creating and showing this application's GUI.
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                createAndShowGUI();
            }
        });
    }

    class ListSelectionHandler implements ListSelectionListener {
        public void valueChanged(ListSelectionEvent e) {
            lsm = (ListSelectionModel) e.getSource();

            int firstIndex = e.getFirstIndex();
            int lastIndex = e.getLastIndex();
            boolean isAdjusting = e.getValueIsAdjusting();
            if (e.getValueIsAdjusting()) {

                swingParameterLabel.removeAll();
                swingParameterLabel.invalidate();
                swingParameterLabel.repaint();

                swingParameterValue.removeAll();
                swingParameterValue.invalidate();
                swingParameterValue.repaint();
                buttonsGeneratedList = null;
                swingSave.removeAll();
                swingSave.repaint();

                if (swingLeftList.getSelectionModel() == e.getSource()) {
                    System.out.println("LEFT LIST !!!!");
                    selectedItemLeftList = lsm.getMaxSelectionIndex();
                    swingConsoleOutput.append("Selected left list index "
                            + lsm.getMaxSelectionIndex());
                    File fileExists = new File(outputTCGPath
                            + testCaseLeftList.get(lsm.getMaxSelectionIndex())
                            + ".xml");
                    System.out.println("Test case path " + outputTCGPath
                            + testCaseLeftList.get(lsm.getMaxSelectionIndex()));
                    if (fileExists.exists()) {
                        initialTCList = CTG.getTagsFromXML(fileExists
                                .getAbsolutePath());
                    }
                    /*
                     * else { initialTCList = CTG.getTagsFromXML("../" +
                     * testCaseLeftList.get(lsm .getMaxSelectionIndex()));
                     * System.out.println("222222222222222222222 " +
                     * testCaseLeftList.get(lsm .getMaxSelectionIndex())); }
                     */
                    swingConsoleOutput.append(newline);

                    int y = 0;
                    int x = 0;
                    buttonsGeneratedList = new ArrayList<JTextField>();

                    for (int i = 0; i < initialTCList.size(); i++) {
                        swingConsoleOutput.append(initialTCList.get(i));
                        swingConsoleOutput.append(newline);

                        JPanel tmpPanel = new JPanel();
                        JTextField tmpButton = new JTextField();
                        tmpPanel.add(tmpButton);
                        tmpButton.setText(initialTCList.get(i));
                        tmpButton.setPreferredSize(new Dimension(200, 30));
                        gBC.fill = GridBagConstraints.BOTH;
                        gBC.weightx = 1;
                        gBC.weighty = 1;
                        gBC.gridx = x;
                        gBC.gridy = y;
                        gBC.gridwidth = 1;
                        gBC.insets = new Insets(2, 2, 2, 2);
                        if (i % 2 == 0) {
                            tmpButton.setEditable(false);
                            swingParameterLabel.add(tmpPanel, gBC);
                        } else {
                            tmpButton.setEditable(true);
                            buttonsGeneratedList.add(tmpButton);
                            swingParameterValue.add(tmpPanel, gBC);

                        }

                        if (x == 1) {
                            x = 0;
                            y++;
                        } else
                            x++;
                    }

                    // Make autosave in future version
                    JButton swingSaveButton = new JButton("Save");
                    swingSaveButton.addActionListener(new ActionListener() {

                        @Override
                        public void actionPerformed(ActionEvent e) {
                            for (int i = 0; i < buttonsGeneratedList.size(); i++) {
                                System.out
                                        .println("????"
                                                + buttonsGeneratedList.get(i)
                                                        .getText());
                            }

                            CTG.saveX(
                                    outputTCGPath
                                            + testCaseLeftList.get(lsm
                                                    .getMaxSelectionIndex())
                                            + ".xml", buttonsGeneratedList);
                            // buttonsGeneratedList.clear();
                        }

                    });
                    // gBC.anchor = GridBagConstraints.SOUTH;
                    // swingSave.add(swingSaveButton);
                    // swingParameterValue.add(swingSaveButton);
                    swingSave.add(swingSaveButton);

                } else {
                    swingConsoleOutput.append("Selected right list index "
                            + lsm.getMaxSelectionIndex());
                    selectedItemRightList = lsm.getMaxSelectionIndex();
                }
                swingConsoleOutput.append(newline);
                swingConsoleOutput
                        .append("----------------------------------------");
                swingConsoleOutput.append(newline);

            }
            swingParameterValue.validate();
            swingParameterLabel.validate();
            swingParameterPane.validate();
        }
    }

    class ButtonAddHandler implements ActionListener {

        @Override
        public void actionPerformed(ActionEvent e) {
            String destName = null;
            for (int i = 0; i < swingRightList.getSelectedValuesList().size(); i++) {
                System.out.println("!!!"
                        + swingRightList.getSelectedValuesList().get(i));

            }
            for (int i = 0; i < swingRightList.getSelectedValuesList().size(); i++) {
                String itemToBeAdded = swingRightList.getSelectedValuesList()
                        .get(i).toString();
                try {
                    destName = itemToBeAdded.split(".xml")[0];
                    File dest = new File(outputTCGPath + destName + ".xml");
                    dest.getParentFile().mkdirs();
                    File src = new File("../" + itemToBeAdded);
                    System.out.println("SRC " + src);
                    System.out.println("DEST " + dest);
                    if (dest.exists()) {
                        for (int j = 0; j < 100; j++) {
                            if (dest.exists()) {
                                destName = itemToBeAdded.split(".xml")[0] + "_"
                                        + j;
                                dest = new File(outputTCGPath + destName
                                        + ".xml");
                            }
                        }
                    }
                    // Files.deleteIfExists(dest.toPath());
                    System.out.println("Destination path" + outputTCGPath
                            + destName + ".xml");
                    Files.copy(new File("../" + itemToBeAdded).toPath(),
                            dest.toPath());

                } catch (IOException e1) {
                    // TODO Auto-generated catch block
                    e1.printStackTrace();
                }

                // listModel.add(listModel.size(), itemToBeAdded);
                // testCaseLeftList.add(itemToBeAdded);

                listModel.add(listModel.size(), destName);
                testCaseLeftList.add(destName);

            }
            swingRightList.clearSelection();
        }
    }

    class ButtonRemoveHandler implements ActionListener {
        ArrayList removedItems = new ArrayList<String>();

        @Override
        public void actionPerformed(ActionEvent e) {
            System.out.println("Remove" + selectedItemLeftList);
            for (int i = 0; i < swingLeftList.getSelectedValuesList().size(); i++) {
                System.out.println("REMOVING "
                        + swingLeftList.getSelectedValuesList().get(i)
                                .toString());
                removedItems.add(swingLeftList.getSelectedValuesList().get(i)
                        .toString());
                testCaseLeftList.remove(swingLeftList.getSelectedValuesList()
                        .get(i).toString());
                System.out.println("File to be deleted "
                        + outputTCGPath
                        + swingLeftList.getSelectedValuesList().get(i)
                                .toString() + ".xml");
                File removeFile = new File(outputTCGPath
                        + swingLeftList.getSelectedValuesList().get(i)
                                .toString() + ".xml");
                removeFile.delete();

            }

            int index[] = swingLeftList.getSelectedIndices();
            System.out.println("size: " + index.length + " "
                    + removedItems.size());
            if ((index.length > 0) && (removedItems.size() > 0)) {

                for (int i = 0; i < removedItems.size(); i++) {
                    System.out.println("???" + removedItems.get(i) + " "
                            + listModel.contains(removedItems.get(i)));
                    listModel.removeElement(removedItems.get(i));
                }
            }
            swingLeftList.clearSelection();
            removedItems.clear();
        }
    }

    class ButtonGenerateCampaignHandler implements ActionListener {

        @Override
        public void actionPerformed(ActionEvent e) {
            CTG.generateCampaign(testCaseLeftList);
            System.out.println("Campaign generated");
        }
    }
}
