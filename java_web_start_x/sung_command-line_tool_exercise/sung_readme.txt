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

5. To launch the app using the link from a web browser,
    1. google-chrome appPage.html.
    2. Click the link.
    3. Save the jnlp file to this directory.
    4. Launch the jnlp file using,
        javaws webstart.jnlp
    5. This should pop-up bunch of security windows and ultimately launch the
        app.
