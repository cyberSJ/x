/**
 * Compile with
 * javac -h . HelloJni.java
 * where -h is for generating .h header file
 * and . is the location of the generated header file.
 * This command creates a .class file and then .h file.
 *
 * Create the C (C++) shared library.
 *
 * Run with
 * java -Djava.library.path=<location of the C library> HelloJni
 */
public class HelloJni
{
    static
    {
        // Resolves to lib.hello.so shared library in Linux.
        System.loadLibrary("hello");
    }

    // Define a native method.
    private native void sayHello();

    public static void main(String[] args)
    {
        new HelloJni().sayHello();
    }
}

