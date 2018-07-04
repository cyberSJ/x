public class JString
{
    static
    {
        System.loadLibrary("jstring");
    }

    private native String say(String msg);

    public static void main(String[] args)
    {
        System.out.println(new JString().say("hello jstring"));
    }
}
