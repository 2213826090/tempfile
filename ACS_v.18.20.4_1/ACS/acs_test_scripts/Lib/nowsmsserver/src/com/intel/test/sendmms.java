// The sendmms class can be used to send an MMS message via NowSMS.
//
// This class supports all of the MMS related parameters for NowSMS which are described at the
// following link:
//
// http://blog.nowsms.com/2008/11/nowsms-php-example-send-mms-message.html
//
// Begin by creating a new sendmms object, specifying the address of the NowSMS server, and a valid username and password
// for a user account ("SMS Users") on the NowSMS Server.
//
//    sendmms mms = new sendmms ("http://127.0.0.1:8800/", "test", "test");
//
// The addparameter method is used to build the MMS message object.
//
// Start by specifying a recipient (this can be a comma delimited list of recipients):
//
//    mms.addparameter ("PhoneNumber", "+9999999999");
//
// Next, add any desired MMS header parameters, such as the message subject, or an optional text part of the message:
//
//    mms.addparameter ("MMSSubject", "This a a test message");
//    mms.addparameter ("MMSText", "test message");  // Optional
//
// Next, add the file objects for the MMS content.
//
//    mms.addparameter ("MMSFile", new File("f:\\temp\\test.jpg"));
//    mms.addparameter ("MMSFile", new File("f:\\temp\\test2.jpg"));
//
//
// The send method submits the MMS message to NowSMS.
//
//    mms.send ();
//
//
// The send method returns a string containing the MMS Message ID assigned for the submitted messages, in the following format:
//
//    MMSMessageID=xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//
//
// Note that a try/catch exception handler must be added around the object.  Here's a complete example:
//
//  try {
//
//    sendmms mms = new sendmms ("http://127.0.0.1:8800/", "test", "test");
//
//    mms.addparameter ("PhoneNumber", "+9999999999");
//
//    mms.addparameter ("MMSSubject", "This a a test message");
//    mms.addparameter ("MMSText", "test message");  // Optional
//
//    mms.addparameter ("MMSFile", new File("f:\\temp\\test.jpg"));
//    mms.addparameter ("MMSFile", new File("f:\\temp\\test2.jpg"));
//
//    mms.send ();
//  }
//  catch(IOException e) {
//    System.out.println("unable to create new url: "+e.getMessage());
//  }
//
//
// Note:  This example script is adapted from the ClientHttpRequest class created by Vlad Patryshev
// For more information on the original ClientHttpRequest script, see:
//
// http://www.devx.com/Java/Article/17679/1954
// http://www.myjavatools.com/
//
//

package com.intel.test;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Random;

/**
 * <p>Title: Client HTTP Request class</p>
 * <p>Description: this class helps to send POST HTTP requests with various form data,
 * including files. Cookies can be added to be included in the request.</p>
 *
 * @author Vlad Patryshev
 * @version 1.0
 */
public class sendmms {
    public static HttpURLConnection connection;
    OutputStream os = null;
    Map cookies = new HashMap();

    protected void connect() throws IOException {
        if (os == null) os = connection.getOutputStream();
    }

    protected void write(char c) throws IOException {
        connect();
        os.write(c);
    }

    protected void write(String s) throws IOException {
        connect();
        os.write(s.getBytes());
    }

    protected void newline() throws IOException {
        connect();
        write("\r\n");
    }

    protected void writeln(String s) throws IOException {
        connect();
        write(s);
        newline();
    }

    private static Random random = new Random();

    protected static String randomString() {
        return Long.toString(random.nextLong(), 36);
    }

    String boundary = "---------------------------" + randomString() + randomString() + randomString();

    private void boundary() throws IOException {
        write("--");
        write(boundary);
    }

    /**
    * Creates a new multipart POST HTTP request on a freshly opened URLConnection
    *
    * @param connection an already open URL connection
    * @throws IOException
    */
    public sendmms(HttpURLConnection connection) throws IOException {
        this.connection = connection;
        connection.setDoOutput(true);
        connection.setRequestProperty("Content-Type",
        "multipart/form-data; boundary=" + boundary);
    }

    /**
    * Creates a new multipart POST HTTP request for a specified URL
    *
    * @param url the URL to send request to
    * @throws IOException
    */
    public sendmms(URL url) throws IOException {
        this((HttpURLConnection)url.openConnection());
    }

    /**
    * Creates a new multipart POST HTTP request for a specified URL string
    *
    * @param urlString the string representation of the URL to send request to
    * @throws IOException
    */
    public sendmms(String urlString) throws IOException {
        this(new URL(urlString));
    }

    public sendmms(String urlString, String user, String password) throws IOException {
        this(new URL(urlString + "?user=" + user + "&password=" + password));
    }

    private void postCookies() {
        StringBuffer cookieList = new StringBuffer();

        for (Iterator i = cookies.entrySet().iterator(); i.hasNext();) {
            Map.Entry entry = (Map.Entry)(i.next());
            cookieList.append(entry.getKey().toString() + "=" + entry.getValue());

            if (i.hasNext()) {
                cookieList.append("; ");
            }
        }
        if (cookieList.length() > 0) {
            connection.setRequestProperty("Cookie", cookieList.toString());
        }
    }

    /**
    * adds a cookie to the requst
    * @param name cookie name
    * @param value cookie value
    * @throws IOException
    */
    public void setCookie(String name, String value) throws IOException {
        cookies.put(name, value);
    }

    /**
    * adds cookies to the request
    * @param cookies the cookie "name-to-value" map
    * @throws IOException
    */
    public void setCookies(Map cookies) throws IOException {
        if (cookies == null) return;
        this.cookies.putAll(cookies);
    }

    /**
    * adds cookies to the request
    * @param cookies array of cookie names and values (cookies[2*i] is a name, cookies[2*i + 1] is a value)
    * @throws IOException
    */
    public void setCookies(String[] cookies) throws IOException {
        if (cookies == null) return;
        for (int i = 0; i < cookies.length - 1; i+=2) {
            setCookie(cookies[i], cookies[i+1]);
        }
    }

    private void writeName(String name) throws IOException {
        newline();
        write("Content-Disposition: form-data; name=\"");
        write(name);
        write('"');
    }

    /**
    * adds a string parameter to the request
    * @param name parameter name
    * @param value parameter value
    * @throws IOException
    */
    public void addparameter(String name, String value) throws IOException {
        boundary();
        writeName(name);
        newline(); newline();
        writeln(value);
    }

    private static void pipe(InputStream in, OutputStream out) throws IOException {
        byte[] buf = new byte[500000];
        int nread;
        int navailable;
        int total = 0;
        synchronized (in) {
            while((nread = in.read(buf, 0, buf.length)) >= 0) {
                out.write(buf, 0, nread);
                total += nread;
            }
        }
        out.flush();
        buf = null;
    }

    /**
    * adds a file parameter to the request
    * @param name parameter name
    * @param filename the name of the file
    * @param is input stream to read the contents of the file from
    * @throws IOException
    */
    public void addparameter(String name, String filename, InputStream is) throws IOException {
        boundary();
        writeName(name);
        write("; filename=\"");
        write(filename);
        write('"');
        newline();
        write("Content-Type: ");
        String type = connection.guessContentTypeFromName(filename);
        if (type == null) type = "application/octet-stream";
        writeln(type);
        newline();
        pipe(is, os);
        newline();
    }

    /**
    * adds a file parameter to the request
    * @param name parameter name
    * @param file the file to upload
    * @throws IOException
    */
    public void addparameter(String name, File file) throws IOException {
        addparameter(name, file.getPath(), new FileInputStream(file));
    }

    /**
    * adds a parameter to the request; if the parameter is a File, the file is uploaded, otherwise the string value of the parameter is passed in the request
    * @param name parameter name
    * @param object parameter value, a File or anything else that can be stringified
    * @throws IOException
    */
    public void addparameter(String name, Object object) throws IOException {
        if (object instanceof File) {
            addparameter(name, (File) object);
        } else {
            addparameter(name, object.toString());
        }
    }

    /**
    * adds parameters to the request
    * @param parameters "name-to-value" map of parameters; if a value is a file, the file is uploaded, otherwise it is stringified and sent in the request
    * @throws IOException
    */
    public void addparameters(Map parameters) throws IOException {
        if (parameters == null) return;
        for (Iterator i = parameters.entrySet().iterator(); i.hasNext();) {
            Map.Entry entry = (Map.Entry)i.next();
            addparameter(entry.getKey().toString(), entry.getValue());
        }
    }

    /**
    * adds parameters to the request
    * @param parameters array of parameter names and values (parameters[2*i] is a name, parameters[2*i + 1] is a value); if a value is a file, the file is uploaded, otherwise it is stringified and sent in the request
    * @throws IOException
    */
    public void addparameters(Object[] parameters) throws IOException {
        if (parameters == null) return;
        for (int i = 0; i < parameters.length - 1; i+=2) {
            addparameter(parameters[i].toString(), parameters[i+1]);
        }
    }

    /**
    * posts the requests to the server, with all the cookies and parameters that were added
    * @return input stream with the server response
    * @throws IOException
    */
    public InputStream post() throws IOException {
        boundary();
        writeln("--");
        os.close();
        return connection.getInputStream();
    }

    /**
    * posts the requests to the server, with all the cookies and parameters that were added before (if any), and with parameters that are passed in the argument
    * @param parameters request parameters
    * @return input stream with the server response
    * @throws IOException
    * @see addparameters
    */
    public InputStream post(Map parameters) throws IOException {
        addparameters(parameters);
        return post();
    }

    /**
    * posts the requests to the server, with all the cookies and parameters that were added before (if any), and with parameters that are passed in the argument
    * @param parameters request parameters
    * @return input stream with the server response
    * @throws IOException
    * @see addparameters
    */
    public InputStream post(Object[] parameters) throws IOException {
        addparameters(parameters);
        return post();
    }

    /**
    * posts the requests to the server, with all the cookies and parameters that were added before (if any), and with cookies and parameters that are passed in the arguments
    * @param cookies request cookies
    * @param parameters request parameters
    * @return input stream with the server response
    * @throws IOException
    * @see addparameters
    * @see setCookies
    */
    public InputStream post(Map cookies, Map parameters) throws IOException {
        setCookies(cookies);
        addparameters(parameters);
        return post();
    }

    /**
    * posts the requests to the server, with all the cookies and parameters that were added before (if any), and with cookies and parameters that are passed in the arguments
    * @param cookies request cookies
    * @param parameters request parameters
    * @return input stream with the server response
    * @throws IOException
    * @see addparameters
    * @see setCookies
    */
    public InputStream post(String[] cookies, Object[] parameters) throws IOException {
        setCookies(cookies);
        addparameters(parameters);
        return post();
    }

    /**
    * post the POST request to the server, with the specified parameter
    * @param name parameter name
    * @param value parameter value
    * @return input stream with the server response
    * @throws IOException
    * @see addparameter
    */
    public InputStream post(String name, Object value) throws IOException {
        addparameter(name, value);
        return post();
    }

    /**
    * post the POST request to the server, with the specified parameters
    * @param name1 first parameter name
    * @param value1 first parameter value
    * @param name2 second parameter name
    * @param value2 second parameter value
    * @return input stream with the server response
    * @throws IOException
    * @see addparameter
    */
    public InputStream post(String name1, Object value1, String name2, Object value2) throws IOException {
        addparameter(name1, value1);
        return post(name2, value2);
    }

    /**
    * post the POST request to the server, with the specified parameters
    * @param name1 first parameter name
    * @param value1 first parameter value
    * @param name2 second parameter name
    * @param value2 second parameter value
    * @param name3 third parameter name
    * @param value3 third parameter value
    * @return input stream with the server response
    * @throws IOException
    * @see addparameter
    */
    public InputStream post(String name1, Object value1, String name2, Object value2, String name3, Object value3) throws IOException {
        addparameter(name1, value1);
        return post(name2, value2, name3, value3);
    }

    /**
    * post the POST request to the server, with the specified parameters
    * @param name1 first parameter name
    * @param value1 first parameter value
    * @param name2 second parameter name
    * @param value2 second parameter value
    * @param name3 third parameter name
    * @param value3 third parameter value
    * @param name4 fourth parameter name
    * @param value4 fourth parameter value
    * @return input stream with the server response
    * @throws IOException
    * @see addparameter
    */
    public InputStream post(String name1, Object value1, String name2, Object value2, String name3, Object value3, String name4, Object value4) throws IOException {
        addparameter(name1, value1);
        return post(name2, value2, name3, value3, name4, value4);
    }

    /**
    * posts a new request to specified URL, with parameters that are passed in the argument
    * @param parameters request parameters
    * @return input stream with the server response
    * @throws IOException
    * @see addparameters
    */
    public static InputStream post(URL url, Map parameters) throws IOException {
        return new sendmms(url).post(parameters);
    }

    /**
    * posts a new request to specified URL, with parameters that are passed in the argument
    * @param parameters request parameters
    * @return input stream with the server response
    * @throws IOException
    * @see addparameters
    */
    public static InputStream post(URL url, Object[] parameters) throws IOException {
        return new sendmms(url).post(parameters);
    }

    /**
    * posts a new request to specified URL, with cookies and parameters that are passed in the argument
    * @param cookies request cookies
    * @param parameters request parameters
    * @return input stream with the server response
    * @throws IOException
    * @see setCookies
    * @see addparameters
    */
    public static InputStream post(URL url, Map cookies, Map parameters) throws IOException {
        return new sendmms(url).post(cookies, parameters);
    }

    /**
    * posts a new request to specified URL, with cookies and parameters that are passed in the argument
    * @param cookies request cookies
    * @param parameters request parameters
    * @return input stream with the server response
    * @throws IOException
    * @see setCookies
    * @see addparameters
    */
    public static InputStream post(URL url, String[] cookies, Object[] parameters) throws IOException {
        return new sendmms(url).post(cookies, parameters);
    }

    /**
    * post the POST request specified URL, with the specified parameter
    * @param name parameter name
    * @param value parameter value
    * @return input stream with the server response
    * @throws IOException
    * @see addparameter
    */
    public static InputStream post(URL url, String name1, Object value1) throws IOException {
        return new sendmms(url).post(name1, value1);
    }

    /**
    * post the POST request to specified URL, with the specified parameters
    * @param name1 first parameter name
    * @param value1 first parameter value
    * @param name2 second parameter name
    * @param value2 second parameter value
    * @return input stream with the server response
    * @throws IOException
    * @see addparameter
    */
    public static InputStream post(URL url, String name1, Object value1, String name2, Object value2) throws IOException {
        return new sendmms(url).post(name1, value1, name2, value2);
    }

    /**
    * post the POST request to specified URL, with the specified parameters
    * @param name1 first parameter name
    * @param value1 first parameter value
    * @param name2 second parameter name
    * @param value2 second parameter value
    * @param name3 third parameter name
    * @param value3 third parameter value
    * @return input stream with the server response
    * @throws IOException
    * @see addparameter
    */
    public static InputStream post(URL url, String name1, Object value1, String name2, Object value2, String name3, Object value3) throws IOException {
        return new sendmms(url).post(name1, value1, name2, value2, name3, value3);
    }

    /**
    * post the POST request to specified URL, with the specified parameters
    * @param name1 first parameter name
    * @param value1 first parameter value
    * @param name2 second parameter name
    * @param value2 second parameter value
    * @param name3 third parameter name
    * @param value3 third parameter value
    * @param name4 fourth parameter name
    * @param value4 fourth parameter value
    * @return input stream with the server response
    * @throws IOException
    * @see addparameter
    */
    public static InputStream post(URL url, String name1, Object value1, String name2, Object value2, String name3, Object value3, String name4, Object value4) throws IOException {
        return new sendmms(url).post(name1, value1, name2, value2, name3, value3, name4, value4);
    }

    public String send () throws IOException  {
        InputStream inputStream = post();
        String returnstring = null;

        String res=connection.getResponseMessage();
        System.out.println("Response Code  ->"+res);

        BufferedReader in = new BufferedReader(new InputStreamReader(inputStream));

        String str;

        while( null != ((str = in.readLine()))) {
            if (str.startsWith("MMSMessageID=")) {
                returnstring = returnstring + str + "\r\n";
                System.out.println(str);
            }
        }
        return returnstring;
    }

}
