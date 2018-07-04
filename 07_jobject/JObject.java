public class JObject
{
    static
    {
        System.loadLibrary("jobject");
    }

    public static void main(String[] args)
    {
        JObject instance = new JObject();
        instance.modifyInstanceVariable();
        instance.intMember = 0;
        instance.stringMember = "";

        System.out.println("modified int member   : " + instance.intMember);
        System.out.println("modified String member: " + instance.stringMember);
    }

    private native void modifyInstanceVariable();

    private int intMember;
    private String stringMember;
}
