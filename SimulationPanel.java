import java.awt.Color;
import java.awt.Graphics;
import javax.swing.JPanel;
import javax.swing.Timer;

public class SimulationPanel extends JPanel {
    private Timer timer;

    public SimulationPanel() {
        setBackground(Color.BLACK);
    }

    public void start() {
        timer = new Timer(16, e -> repaint()); // about 60 FPS
        timer.start();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        // background
        g.setColor(Color.DARK_GRAY);
        g.fillRect(0, 0, getWidth(), getHeight());

        // test circle
        g.setColor(Color.CYAN);
        int diameter = 60;
        int x = getWidth() / 2 - diameter / 2;
        int y = getHeight() / 2 - diameter / 2;
        g.fillOval(x, y, diameter, diameter);
    }
}