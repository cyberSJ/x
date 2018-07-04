public class JArray
{
    static
    {
        System.loadLibrary("jarray");
    }

    private native double[] sumAndAverage(int[] numbers);

    public static void main(String[] args)
    {
        int[] numbers = {1, 2, 4};
        double[] results = new JArray().sumAndAverage(numbers);
        System.out.println("sum    : " + results[0]);
        System.out.println("average: " + results[1]);
    }
}
