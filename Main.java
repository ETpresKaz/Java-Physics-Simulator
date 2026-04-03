import javax.swing.JFrame;
import javax.swing.SwingUtilities;

public class Main {
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("2D Physics Sim");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setSize(1000, 700);
            frame.setLocationRelativeTo(null);

            SimulationPanel panel = new SimulationPanel();
            frame.add(panel);

            frame.setVisible(true);
            panel.requestFocusInWindow();
            panel.start();
        });
    }
}