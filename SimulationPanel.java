import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import javax.swing.JPanel;
import javax.swing.Timer;

public class SimulationPanel extends JPanel {
    private static final double GRAVITY = 900.0;
    private static final double RESTITUTION = 0.88;
    private static final double WALL_FRICTION = 0.995;
    private static final double AIR_DRAG = 0.999;
    private static final int TARGET_FPS = 60;
    private static final int INITIAL_BALLS = 18;

    private final List<Ball> balls = new ArrayList<>();
    private final Random random = new Random();
    private Timer timer;
    private long lastTime;
    private boolean paused = false;
    private boolean showHUD = true;

    public SimulationPanel() {
        setBackground(Color.BLACK);
        setFocusable(true);

        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                switch (e.getKeyCode()) {
                    case KeyEvent.VK_SPACE -> paused = !paused;
                    case KeyEvent.VK_R -> resetSimulation();
                    case KeyEvent.VK_C -> balls.clear();
                    case KeyEvent.VK_H -> showHUD = !showHUD;
                    case KeyEvent.VK_B -> spawnBall(getWidth() / 2.0, 80.0);
                }
            }
        });

        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                spawnBall(e.getX(), e.getY());
                requestFocusInWindow();
            }
        });
    }

    public void start() {
        resetSimulation();
        lastTime = System.nanoTime();
        timer = new Timer(1000 / TARGET_FPS, e -> gameLoop());
        timer.start();
    }

    private void gameLoop() {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1_000_000_000.0;
        lastTime = now;

        dt = Math.min(dt, 0.025);

        if (!paused) {
            updatePhysics(dt);
        }

        repaint();
    }

    private void resetSimulation() {
        balls.clear();

        int width = Math.max(getWidth(), 1000);
        int height = Math.max(getHeight(), 700);

        for (int i = 0; i < INITIAL_BALLS; i++) {
            double radius = 12 + random.nextDouble() * 18;
            double x = 80 + random.nextDouble() * Math.max(1, width - 160);
            double y = 40 + random.nextDouble() * Math.max(1, height / 2.0 - 60);
            double vx = -180 + random.nextDouble() * 360;
            double vy = -50 + random.nextDouble() * 120;
            double mass = radius * radius * 0.08;
            Color color = new Color(
                60 + random.nextInt(196),
                60 + random.nextInt(196),
                60 + random.nextInt(196)
            );
            balls.add(new Ball(x, y, vx, vy, radius, mass, color));
        }
    }

    private void spawnBall(double x, double y) {
        double radius = 10 + random.nextDouble() * 20;
        double vx = -220 + random.nextDouble() * 440;
        double vy = -240 - random.nextDouble() * 120;
        double mass = radius * radius * 0.08;
        Color color = new Color(
            100 + random.nextInt(156),
            100 + random.nextInt(156),
            100 + random.nextInt(156)
        );
        balls.add(new Ball(x, y, vx, vy, radius, mass, color));
    }

    private void updatePhysics(double dt) {
        for (Ball ball : balls) {
            ball.vy += GRAVITY * dt;
            ball.vx *= AIR_DRAG;
            ball.vy *= AIR_DRAG;
            ball.x += ball.vx * dt;
            ball.y += ball.vy * dt;
            handleWallCollision(ball);
        }

        for (int i = 0; i < balls.size(); i++) {
            for (int j = i + 1; j < balls.size(); j++) {
                resolveBallCollision(balls.get(i), balls.get(j));
            }
        }
    }

    private void handleWallCollision(Ball ball) {
        int width = getWidth();
        int height = getHeight();

        if (ball.x - ball.radius < 0) {
            ball.x = ball.radius;
            ball.vx = -ball.vx * RESTITUTION;
        } else if (ball.x + ball.radius > width) {
            ball.x = width - ball.radius;
            ball.vx = -ball.vx * RESTITUTION;
        }

        if (ball.y - ball.radius < 0) {
            ball.y = ball.radius;
            ball.vy = -ball.vy * RESTITUTION;
        } else if (ball.y + ball.radius > height) {
            ball.y = height - ball.radius;
            ball.vy = -ball.vy * RESTITUTION;
            ball.vx *= WALL_FRICTION;

            if (Math.abs(ball.vy) < 10) {
                ball.vy = 0;
            }
        }
    }

    private void resolveBallCollision(Ball a, Ball b) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double distanceSquared = dx * dx + dy * dy;
        double minDistance = a.radius + b.radius;

        if (distanceSquared == 0) {
            dx = 0.01;
            dy = 0.01;
            distanceSquared = dx * dx + dy * dy;
        }

        if (distanceSquared > minDistance * minDistance) {
            return;
        }

        double distance = Math.sqrt(distanceSquared);
        double nx = dx / distance;
        double ny = dy / distance;

        double overlap = minDistance - distance;
        double totalMass = a.mass + b.mass;

        a.x -= nx * overlap * (b.mass / totalMass);
        a.y -= ny * overlap * (b.mass / totalMass);
        b.x += nx * overlap * (a.mass / totalMass);
        b.y += ny * overlap * (a.mass / totalMass);

        double rvx = b.vx - a.vx;
        double rvy = b.vy - a.vy;
        double velocityAlongNormal = rvx * nx + rvy * ny;

        if (velocityAlongNormal > 0) {
            return;
        }

        double e = 0.92;
        double impulseMagnitude = -(1 + e) * velocityAlongNormal;
        impulseMagnitude /= (1.0 / a.mass) + (1.0 / b.mass);

        double impulseX = impulseMagnitude * nx;
        double impulseY = impulseMagnitude * ny;

        a.vx -= impulseX / a.mass;
        a.vy -= impulseY / a.mass;
        b.vx += impulseX / b.mass;
        b.vy += impulseY / b.mass;

        double tx = -ny;
        double ty = nx;
        double tangentVelocity = rvx * tx + rvy * ty;
        double frictionImpulseMagnitude = -tangentVelocity;
        frictionImpulseMagnitude /= (1.0 / a.mass) + (1.0 / b.mass);
        frictionImpulseMagnitude *= 0.03;

        double frictionX = frictionImpulseMagnitude * tx;
        double frictionY = frictionImpulseMagnitude * ty;

        a.vx -= frictionX / a.mass;
        a.vy -= frictionY / a.mass;
        b.vx += frictionX / b.mass;
        b.vy += frictionY / b.mass;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        drawBackground(g2);
        drawBalls(g2);

        if (showHUD) {
            drawHUD(g2);
        }
    }

    private void drawBackground(Graphics2D g2) {
        g2.setColor(new Color(18, 18, 24));
        g2.fillRect(0, 0, getWidth(), getHeight());

        g2.setColor(new Color(35, 35, 45));
        for (int x = 0; x < getWidth(); x += 40) {
            g2.drawLine(x, 0, x, getHeight());
        }
        for (int y = 0; y < getHeight(); y += 40) {
            g2.drawLine(0, y, getWidth(), y);
        }

        g2.setColor(new Color(70, 70, 90));
        g2.setStroke(new BasicStroke(2f));
        g2.drawLine(0, getHeight() - 1, getWidth(), getHeight() - 1);
    }

    private void drawBalls(Graphics2D g2) {
        for (Ball ball : balls) {
            int x = (int) Math.round(ball.x - ball.radius);
            int y = (int) Math.round(ball.y - ball.radius);
            int diameter = (int) Math.round(ball.radius * 2);

            g2.setColor(ball.color);
            g2.fillOval(x, y, diameter, diameter);

            g2.setColor(new Color(255, 255, 255, 90));
            g2.fillOval(x + diameter / 5, y + diameter / 5, Math.max(4, diameter / 4), Math.max(4, diameter / 4));

            g2.setColor(Color.BLACK);
            g2.drawOval(x, y, diameter, diameter);
        }
    }

    private void drawHUD(Graphics2D g2) {
        g2.setFont(new Font("Monospaced", Font.PLAIN, 14));
        g2.setColor(new Color(0, 0, 0, 150));
        g2.fillRoundRect(12, 12, 420, 122, 16, 16);

        g2.setColor(Color.WHITE);
        g2.drawString("2D Physics Simulator", 24, 36);
        g2.drawString("Balls: " + balls.size(), 24, 58);
        g2.drawString("Left click = spawn ball", 24, 80);
        g2.drawString("Space = pause | R = reset | C = clear | B = spawn center | H = hide HUD", 24, 102);
        g2.drawString("Status: " + (paused ? "PAUSED" : "RUNNING"), 24, 124);
    }

    private static class Ball {
        double x;
        double y;
        double vx;
        double vy;
        double radius;
        double mass;
        Color color;

        Ball(double x, double y, double vx, double vy, double radius, double mass, Color color) {
            this.x = x;
            this.y = y;
            this.vx = vx;
            this.vy = vy;
            this.radius = radius;
            this.mass = mass;
            this.color = color;
        }
    }
}