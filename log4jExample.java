// Code obtained from
// https://www.tutorialspoint.com/log4j/log4j_logging_files.htm
// Make sure the log4j.properties path is in the PATH environment variable.
// Assumes the log4j jar is in the CLASSPATH.
// Then to run this:
// 1. javac log4jExample.java
// 2. java log4jExample
import org.apache.log4j.Logger;

import java.io.*;
import java.sql.SQLException;
import java.util.*;

public class log4jExample
{
    static Logger log = Logger.getLogger(log4jExample.class.getName());

    public static void main(String[] args) throws IOException, SQLException
    {
        System.out.println("Hello log4j");

        log.debug("Hello debug");
        log.info("Hello info");
    }
}
