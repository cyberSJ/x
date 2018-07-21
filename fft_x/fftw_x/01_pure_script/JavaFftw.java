import jfftw.complex.Plan;

public class JavaFftw
{
    public static void main(String[] args)
    {
        // Implementing the one-shift in time domain illustrated in this site:
        // https://www.developer.com/java/other/article.php/3457251/Fun-with-Java-Understanding-the-Fast-Fourier-Transform-FFT-Algorithm.htm
        System.out.println("Hello JFFTW");

        // Number of complex samples.
        int N = 16;
        Plan plan = new Plan(N);

        // The input complex samples
        double[] in = { 0, 0,
                        0, 0,
                        2, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0,
                        0, 0 };

        // Perform FFT.
        double[] out = plan.transform(in);
        System.out.println("out length: " + out.length);

        // print the real values
        System.out.println("real values: ");
        for (int i = 0; i < 2*N; i += 2)
        {
            System.out.println(out[i] + " ");
        }
        System.out.println();

        // print the imaginary values
        System.out.println("imag values: ");
        for (int i = 1; i < 2*N; i += 2)
        {
            System.out.println(out[i] + " ");
        }
        System.out.println();
    }
}
