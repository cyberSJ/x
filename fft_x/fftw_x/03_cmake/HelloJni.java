import jfftw.complex.Plan;

public class HelloJni
{
    public static void main(String[] args)
    {
        System.out.println("Hello JNI");

        int N = 16;
        Plan plan = new Plan(N);

        double[] input = {
            0.0, 0.0,
            0.0, 0.0,
            2.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0};

        double[] output = plan.transform(input);

        System.out.println("real = ");
        for (int i = 0; i < 2 * N; i += 2)
        {
            System.out.println(output[i]);
        }
        System.out.println();

        System.out.println("imag = ");
        for (int i = 1; i < 2 * N; i += 2)
        {
            System.out.println(output[i]);
        }
    }
}
