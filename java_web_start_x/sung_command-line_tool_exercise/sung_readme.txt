This sung_temp directory is just for learning manual jar packaging and launching
the jnlp file using javaws command-line tool.

1. This sung_temp directory has .class files, which is pre-compiled code using
gradle.

2. To Create a jar using the .class files,
    jar cvfm sung.jar manifest_modification.txt com/cybersj/jws/MyTopJPanel.class com/cybersj/jws/MyTopJPanel$1.class
    
3. To view the content of the jar
    jar tf sung.jar

4. To view the content of the manifest of the jar
    unzip -p sung.jar META-INF/MANIFEST.MF
    
5. To sign the jar (required when using JWS),
    jarsigner -keystore /home/cyber/keystore_cyber -tsa http://sha256timestamp.ws.symantec.com/sha256/timestamp -signedjar sung_signed.jar sung.jar cyber

6. To launch the app using command-line javaws,
    1. Launch the jnlp file using,
        javaws webstart.jnlp
    2. This should pop-up bunch of security windows and ultimately launch the
        app.

7. To launch the app in a remote client,
    1. Install apache2 server in the server machine, if not already setup.
    2. Set up a virtual host (sungexample.com) in the apache2 server directory
        tree.
    3. Assuming set up is done correctly, copy the index.html to
        /var/www/sungexample.com/html/. This is the web root location used by
        apache2.
    4. Copy the sungexample.com.conf file to /etc/apache2/sites-available/. This
        allows Apache2 to refer to the web root location.
    5. Copy sung_signed.jar and webstart.jnlp files to
        /var/www/sungexample.com/html/. This allows the webpage to refer to the
        application resources.
    6. Restart the apache2:
        sudo systemcl restart apache2
    7. Install javaws in the client machine, if not already installed.
    8. Open a web browser (in a remote machine) and navigate to
        http://<ip of server>
    9. Click the link the browser to launch the app.
