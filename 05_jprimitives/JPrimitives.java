public class JPrimitives
{
    static
    {
        System.loadLibrary("jprimitives");
    }

    private native double average(int n1, int n2);

    public static void main(String[] args)
    {
        JPrimitives Prim = new JPrimitives();

        System.out.println("average: " + Prim.average(3, 2));
    }
}
