package my_jni;

public class HelloJni
{
    static
    {
        System.loadLibrary("hello");
    }

    private native void sayHello();

    static public void main(String[] args)
    {
        new HelloJni().sayHello();
    }
}
