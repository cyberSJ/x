// Assumes the log4j jar is in the CLASSPATH.
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
