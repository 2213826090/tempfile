import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JTextField;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerConfigurationException;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.w3c.dom.Comment;
import org.xml.sax.SAXException;

public class CTGenerator {
    Document dom;
    ArrayList<String> tcList = new ArrayList();

    public ArrayList<String> importTCList() {
        BufferedReader br;
        try {
            br = new BufferedReader(new FileReader("../TCList.txt"));

            String line;
            while ((line = br.readLine()) != null) {
                // process the line.
                System.out.println(line);
                tcList.add(line);
            }
            br.close();
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        return tcList;
    }

    public ArrayList<String> getTagsFromXML(String url) {

        ArrayList<String> returnList = new ArrayList();

        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        dbf.setIgnoringComments(false);
        try {

            // Using factory get an instance of document builder
            DocumentBuilder db = dbf.newDocumentBuilder();
            // parse using builder to get DOM representation of the XML file
            dom = db.parse(url);
            Element docEle = dom.getDocumentElement();

            NodeList nln = docEle.getElementsByTagName("Parameter");
            if (nln != null && nln.getLength() > 0) {
                for (int i = 0; i < nln.getLength(); i++) {
                    Element el = (Element) nln.item(i);
                    returnList.add(el.getElementsByTagName("Name").item(0)
                            .getTextContent());
                    returnList.add(el.getElementsByTagName("Value").item(0)
                            .getTextContent());
                    System.out.println("Name = "
                            + el.getElementsByTagName("Name").item(0)
                                    .getTextContent());
                    System.out.println("Value = "
                            + el.getElementsByTagName("Value").item(0)
                                    .getTextContent());
                }
            }

        } catch (ParserConfigurationException pce) {
            pce.printStackTrace();
        } catch (SAXException se) {
            se.printStackTrace();
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

        return returnList;
    }

    public void saveX(String url, ArrayList<JTextField> textFiledList) {
        ArrayList<String> returnList = new ArrayList();

        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        dbf.setIgnoringComments(false);
        DocumentBuilder db;
        try {
            db = dbf.newDocumentBuilder();
            dom = db.parse(url);
        } catch (ParserConfigurationException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        } catch (SAXException | IOException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }
        Element docEle = dom.getDocumentElement();

        NodeList nln = docEle.getElementsByTagName("Parameter");
        if (nln != null && nln.getLength() > 0) {
            for (int i = 0; i < nln.getLength(); i++) {
                Element el = (Element) nln.item(i);
                returnList.add(el.getElementsByTagName("Name").item(0)
                        .getTextContent());
                returnList.add(el.getElementsByTagName("Value").item(0)
                        .getTextContent());
                el.getElementsByTagName("Value").item(0)
                        .setTextContent(textFiledList.get(i).getText());
                System.out.println("Name = "
                        + el.getElementsByTagName("Name").item(0)
                                .getTextContent());
                System.out.println("Value = "
                        + el.getElementsByTagName("Value").item(0)
                                .getTextContent());
                TransformerFactory transformerFactory = TransformerFactory
                        .newInstance();
                Transformer transformer;
                System.out.println("URL" + url);

                try {
                    transformer = transformerFactory.newTransformer();

                    DOMSource source = new DOMSource(dom);
                    StreamResult result = new StreamResult(new File(url));
                    transformer.transform(source, result);
                } catch (TransformerException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }
    }

    public static void generateCampaign(ArrayList<String> finalTcList) {
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        dbf.setIgnoringComments(false);
        DocumentBuilder db;
        Document dom = null;
        try {
            db = dbf.newDocumentBuilder();
            dom = db.parse("../CampaignTemplate.xml");
        } catch (ParserConfigurationException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        } catch (SAXException | IOException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }
        Element docEle = dom.getDocumentElement();

        NodeList nln = docEle.getElementsByTagName("TestCases");
        if (nln != null && nln.getLength() > 0) {
            for (int i = 0; i < finalTcList.size(); i++) {
                Element testCaseNode = dom.createElement("TestCase");
                testCaseNode.setAttribute("Id", finalTcList.get(i));
                nln.item(0).appendChild(testCaseNode);
                nln.item(0).appendChild(dom.createTextNode("\n\t"));
            }
            TransformerFactory transformerFactory = TransformerFactory
                    .newInstance();
            Transformer transformer;

            try {
                transformer = transformerFactory.newTransformer();

                DOMSource source = new DOMSource(dom);
                StreamResult result = new StreamResult(new File(
                        "../Campaign.xml"));
                transformer.transform(source, result);
            } catch (TransformerException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

}
