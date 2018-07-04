public class HelloJni
{
    static
    {
        System.loadLibrary("hello");
    }
    
    private native void sayHello();
    
    public static void main(String[] args)
    {
        new HelloJni().sayHello();
    }

}
