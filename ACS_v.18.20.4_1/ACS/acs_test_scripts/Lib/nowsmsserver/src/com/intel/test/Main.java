package com.intel.test;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

public class Main {

    public static void main(String[] args) throws IOException {

        /* Required mandatory IN args are:
        *   <url> (url to Now SMS server with following format: "http://ServerIP:Port"),
        *   <user> (login set in "SMS User" account credential),
        *   <password> (password set in "SMS User" account credential),
        *   <from_number> (must be phone numbers, ex: "8960"),
        *   <destination_number> (idem, ex: "0123456789"),
        *   <subject>,
        *   <text or empty string>,
        *   <mms_attached_file or empty string> (attached file path).
        *
        * This arguments are use to construct 'sendmms' object. Example of use:
        * java -jar sendmms.jar "http://10.102.162.187:8800" "telephony" "telephony"
        * "0123456789" "MMS subject" "This is my MMS text" "/home/lab/20070310225103_superman_g.jpg"
        *
        * Notes: MMS parameters are [Name - (default value)]:
        * ---------------------------------------------------
        * [MMS part]
        * MMSHEADERS - ()
        * MMSFrom - ()
        * MMSSubject - ()
        * MMSText - ()
        * MMSFile - ()
        * MMSBCC - (No)
        * MMSDeliveryReport - (No)
        * MMSReadReport - (No)
        * MMSPriority - (Normal)
        * MMSMessageClass - (Personal)
        * MMSForwardLock - (No)
        *
        * [DRM part]
        * DRMRestrict - (No)
        * DRMRestrictTextXML - ()
        * DRMConstraintCount - ()
        * DRMConstraintStart - ()
        * DRMConstraintEnd - ()
        * DRMConstraintInterval - ()
        *
        * [Others]
        * PhoneNumber - ()
        * Submit - (Submit)
        */

        try {
            // Debug log
            System.out.println(Arrays.toString(args));

            // Init variables with arguments
            String url = args[0];
            String user= args[1];
            String password = args[2];
            String from_number = args[3];
            String destination_number = args[4];
            String subject = args[5];
            String text = args[6];// Could be empty string ""
            String mms_attached_file_test = "";
            mms_attached_file_test = args[7];// Could be empty string ""

            // Create mms object
            sendmms mms = new sendmms (url, user, password);

            // Add sender number
            mms.addparameter ("MMSFrom", from_number);

            // Set main MMS parameters:
            mms.addparameter ("PhoneNumber", destination_number);
            mms.addparameter ("MMSSubject", subject);
            mms.addparameter ("MMSText", text);//Text is optional (could be ""), but parameter must be set.
            if (!mms_attached_file_test.isEmpty()) {
                mms.addparameter ("MMSFile", new File(mms_attached_file_test));//Attach File is optional (could be "")
            }

            // Set other default MMS parameters with default values
            mms.addparameter ("MMSBCC", "No");
            mms.addparameter ("MMSDeliveryReport", "No");
            mms.addparameter ("MMSReadReport", "No");
            mms.addparameter ("MMSPriority", "Normal");
            mms.addparameter ("MMSMessageClass", "Personal");
            mms.addparameter ("MMSForwardLock", "No");
            mms.addparameter ("DRMRestrict", "No");

            // Set last parameter
            mms.addparameter ("Submit", "Submit");

            // Send object to Now SMS server
            mms.send ();
        }
        catch(IOException e) {
            System.out.println("unable to create new url: "+e.getMessage());
            System.out.println("required parameters: <url> <user> <password> <from_number> <destination_number> <subject> <text or empty string> <Null or mms_attached_file1;mms_attached_file2;...>");
        }
    }

}
