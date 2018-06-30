public class HelloJni
{
    static
    {
        // Load libhello.so in Linux.
        System.loadLibrary("hello");
    }

    private native void sayHello();

    public static void main(String[] args)
    {
        new HelloJni().sayHello();
    }
}
