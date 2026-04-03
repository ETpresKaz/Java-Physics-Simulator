import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
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
    private static final double RESTITUTION = 0.84;
    private static final double WALL_FRICTION = 0.992;
    private static final double AIR_DRAG = 0.999;
    private static final int TARGET_FPS = 60;
    private static final int INITIAL_BODIES = 14;

    private final List<Body> bodies = new ArrayList<>();
    private final Random random = new Random();

    private Timer timer;
    private long lastTime;

    private boolean paused = false;
    private boolean showHUD = true;
    private boolean showVelocityVectors = false;
    private boolean showAccelerationVectors = false;
    private boolean showAABBs = false;

    private SpawnMode currentSpawnMode = SpawnMode.CIRCLE;
    private Body selectedBody = null;

    public SimulationPanel() {
        setBackground(Color.BLACK);
        setFocusable(true);

        addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                switch (e.getKeyCode()) {
                    case KeyEvent.VK_SPACE -> paused = !paused;
                    case KeyEvent.VK_R -> resetSimulation();
                    case KeyEvent.VK_C -> {
                        bodies.clear();
                        selectedBody = null;
                    }
                    case KeyEvent.VK_H -> showHUD = !showHUD;
                    case KeyEvent.VK_V -> showVelocityVectors = !showVelocityVectors;
                    case KeyEvent.VK_A -> showAccelerationVectors = !showAccelerationVectors;
                    case KeyEvent.VK_X -> showAABBs = !showAABBs;
                    case KeyEvent.VK_1 -> currentSpawnMode = SpawnMode.CIRCLE;
                    case KeyEvent.VK_2 -> currentSpawnMode = SpawnMode.BOX;
                    case KeyEvent.VK_B -> spawnBody(getWidth() / 2.0, 90.0, currentSpawnMode);
                }
                repaint();
            }
        });

        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                requestFocusInWindow();

                if (paused) {
                    selectedBody = findBodyAt(e.getPoint());
                } else {
                    spawnBody(e.getX(), e.getY(), currentSpawnMode);
                }

                repaint();
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
        bodies.clear();
        selectedBody = null;

        int width = Math.max(getWidth(), 1000);
        int height = Math.max(getHeight(), 700);

        for (int i = 0; i < INITIAL_BODIES; i++) {
            SpawnMode mode = (i % 2 == 0) ? SpawnMode.CIRCLE : SpawnMode.BOX;
            double x = 100 + random.nextDouble() * Math.max(1, width - 200);
            double y = 50 + random.nextDouble() * Math.max(1, height / 2.0 - 100);
            spawnBody(x, y, mode);
        }
    }

    private void spawnBody(double x, double y, SpawnMode mode) {
        Body body = new Body();
        body.shape = mode;
        body.x = x;
        body.y = y;
        body.vx = -220 + random.nextDouble() * 440;
        body.vy = -180 - random.nextDouble() * 160;
        body.ax = 0;
        body.ay = GRAVITY;
        body.color = new Color(
            70 + random.nextInt(160),
            70 + random.nextInt(160),
            70 + random.nextInt(160)
        );

        if (mode == SpawnMode.CIRCLE) {
            body.radius = 12 + random.nextDouble() * 20;
            body.width = body.radius * 2.0;
            body.height = body.radius * 2.0;
            body.mass = Math.PI * body.radius * body.radius * 0.020;
        } else {
            body.width = 24 + random.nextDouble() * 28;
            body.height = 24 + random.nextDouble() * 28;
            body.radius = Math.max(body.width, body.height) * 0.5;
            body.mass = body.width * body.height * 0.016;
        }

        bodies.add(body);
    }

    private void updatePhysics(double dt) {
        for (Body body : bodies) {
            body.ax = 0;
            body.ay = GRAVITY;

            body.vx += body.ax * dt;
            body.vy += body.ay * dt;

            body.vx *= AIR_DRAG;
            body.vy *= AIR_DRAG;

            body.x += body.vx * dt;
            body.y += body.vy * dt;

            handleWallCollision(body);
        }

        for (int i = 0; i < bodies.size(); i++) {
            for (int j = i + 1; j < bodies.size(); j++) {
                resolveCollision(bodies.get(i), bodies.get(j));
            }
        }
    }

    private void handleWallCollision(Body body) {
        if (body.getLeft() < 0) {
            body.x = body.halfWidth();
            body.vx = -body.vx * RESTITUTION;
        } else if (body.getRight() > getWidth()) {
            body.x = getWidth() - body.halfWidth();
            body.vx = -body.vx * RESTITUTION;
        }

        if (body.getTop() < 0) {
            body.y = body.halfHeight();
            body.vy = -body.vy * RESTITUTION;
        } else if (body.getBottom() > getHeight()) {
            body.y = getHeight() - body.halfHeight();
            body.vy = -body.vy * RESTITUTION;
            body.vx *= WALL_FRICTION;

            if (Math.abs(body.vy) < 12.0) {
                body.vy = 0.0;
            }
        }
    }

    private void resolveCollision(Body a, Body b) {
        if (a.shape == SpawnMode.CIRCLE && b.shape == SpawnMode.CIRCLE) {
            resolveCircleCircle(a, b);
        } else {
            resolveAABBCollision(a, b);
        }
    }

    private void resolveCircleCircle(Body a, Body b) {
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

        separateBodies(a, b, nx, ny, overlap);
        applyImpulse(a, b, nx, ny);
    }

    private void resolveAABBCollision(Body a, Body b) {
        double overlapX = Math.min(a.getRight(), b.getRight()) - Math.max(a.getLeft(), b.getLeft());
        double overlapY = Math.min(a.getBottom(), b.getBottom()) - Math.max(a.getTop(), b.getTop());

        if (overlapX <= 0 || overlapY <= 0) {
            return;
        }

        double nx;
        double ny;
        double overlap;

        if (overlapX < overlapY) {
            overlap = overlapX;
            nx = (a.x < b.x) ? 1.0 : -1.0;
            ny = 0.0;
        } else {
            overlap = overlapY;
            nx = 0.0;
            ny = (a.y < b.y) ? 1.0 : -1.0;
        }

        separateBodies(a, b, nx, ny, overlap);
        applyImpulse(a, b, nx, ny);
    }

    private void separateBodies(Body a, Body b, double nx, double ny, double overlap) {
        double totalMass = a.mass + b.mass;
        if (totalMass <= 0) {
            return;
        }

        double correction = overlap + 0.01;
        a.x -= nx * correction * (b.mass / totalMass);
        a.y -= ny * correction * (b.mass / totalMass);
        b.x += nx * correction * (a.mass / totalMass);
        b.y += ny * correction * (a.mass / totalMass);
    }

    private void applyImpulse(Body a, Body b, double nx, double ny) {
        double rvx = b.vx - a.vx;
        double rvy = b.vy - a.vy;
        double velocityAlongNormal = rvx * nx + rvy * ny;

        if (velocityAlongNormal > 0) {
            return;
        }

        double impulseMagnitude = -(1.0 + RESTITUTION) * velocityAlongNormal;
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
        frictionImpulseMagnitude *= 0.06;

        double frictionX = frictionImpulseMagnitude * tx;
        double frictionY = frictionImpulseMagnitude * ty;

        a.vx -= frictionX / a.mass;
        a.vy -= frictionY / a.mass;
        b.vx += frictionX / b.mass;
        b.vy += frictionY / b.mass;
    }

    private Body findBodyAt(Point point) {
        for (int i = bodies.size() - 1; i >= 0; i--) {
            Body body = bodies.get(i);
            if (body.contains(point.x, point.y)) {
                return body;
            }
        }
        return null;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        drawBackground(g2);
        drawBodies(g2);
        drawDebugOverlays(g2);

        if (showHUD) {
            drawHUD(g2);
        }

        if (paused && selectedBody != null) {
            drawSelectedBodyPanel(g2, selectedBody);
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

    private void drawBodies(Graphics2D g2) {
        for (Body body : bodies) {
            if (body == selectedBody && paused) {
                g2.setColor(new Color(255, 255, 255, 40));
                g2.fillRect((int) Math.round(body.getLeft() - 6), (int) Math.round(body.getTop() - 6), (int) Math.round(body.width + 12), (int) Math.round(body.height + 12));
            }

            g2.setColor(body.color);
            if (body.shape == SpawnMode.CIRCLE) {
                g2.fillOval((int) Math.round(body.getLeft()), (int) Math.round(body.getTop()), (int) Math.round(body.width), (int) Math.round(body.height));
                g2.setColor(new Color(255, 255, 255, 90));
                g2.fillOval((int) Math.round(body.getLeft() + body.width * 0.2), (int) Math.round(body.getTop() + body.height * 0.2), Math.max(4, (int) Math.round(body.width * 0.22)), Math.max(4, (int) Math.round(body.height * 0.22)));
                g2.setColor(Color.BLACK);
                g2.drawOval((int) Math.round(body.getLeft()), (int) Math.round(body.getTop()), (int) Math.round(body.width), (int) Math.round(body.height));
            } else {
                g2.fillRect((int) Math.round(body.getLeft()), (int) Math.round(body.getTop()), (int) Math.round(body.width), (int) Math.round(body.height));
                g2.setColor(new Color(255, 255, 255, 70));
                g2.fillRect((int) Math.round(body.getLeft() + 4), (int) Math.round(body.getTop() + 4), Math.max(4, (int) Math.round(body.width * 0.28)), Math.max(4, (int) Math.round(body.height * 0.28)));
                g2.setColor(Color.BLACK);
                g2.drawRect((int) Math.round(body.getLeft()), (int) Math.round(body.getTop()), (int) Math.round(body.width), (int) Math.round(body.height));
            }
        }
    }

    private void drawDebugOverlays(Graphics2D g2) {
        for (Body body : bodies) {
            if (showAABBs) {
                g2.setColor(new Color(255, 80, 80, 180));
                g2.drawRect((int) Math.round(body.getLeft()), (int) Math.round(body.getTop()), (int) Math.round(body.width), (int) Math.round(body.height));
            }

            if (showVelocityVectors) {
                drawVector(g2, body.x, body.y, body.vx * 0.18, body.vy * 0.18, new Color(80, 220, 255));
            }

            if (showAccelerationVectors) {
                drawVector(g2, body.x, body.y, body.ax * 0.05, body.ay * 0.05, new Color(255, 210, 80));
            }
        }
    }

    private void drawVector(Graphics2D g2, double x, double y, double dx, double dy, Color color) {
        int x1 = (int) Math.round(x);
        int y1 = (int) Math.round(y);
        int x2 = (int) Math.round(x + dx);
        int y2 = (int) Math.round(y + dy);

        g2.setColor(color);
        g2.setStroke(new BasicStroke(2f));
        g2.drawLine(x1, y1, x2, y2);

        double angle = Math.atan2(dy, dx);
        int arrowSize = 8;
        int ax1 = (int) Math.round(x2 - arrowSize * Math.cos(angle - Math.PI / 6));
        int ay1 = (int) Math.round(y2 - arrowSize * Math.sin(angle - Math.PI / 6));
        int ax2 = (int) Math.round(x2 - arrowSize * Math.cos(angle + Math.PI / 6));
        int ay2 = (int) Math.round(y2 - arrowSize * Math.sin(angle + Math.PI / 6));

        g2.drawLine(x2, y2, ax1, ay1);
        g2.drawLine(x2, y2, ax2, ay2);
    }

    private void drawHUD(Graphics2D g2) {
        g2.setFont(new Font("Monospaced", Font.PLAIN, 14));
        g2.setColor(new Color(0, 0, 0, 150));
        g2.fillRoundRect(12, 12, 690, 142, 16, 16);

        g2.setColor(Color.WHITE);
        g2.drawString("Advanced 2D Physics Simulator", 24, 34);
        g2.drawString("Bodies: " + bodies.size() + " | Status: " + (paused ? "PAUSED" : "RUNNING") + " | Spawn: " + currentSpawnMode.label, 24, 56);
        g2.drawString("Running click = spawn selected shape | Paused click = inspect body", 24, 78);
        g2.drawString("1 = circle | 2 = box | B = spawn center | V = velocity | A = acceleration | X = AABB", 24, 100);
        g2.drawString("Space = pause | R = reset | C = clear | H = hide HUD", 24, 122);
        g2.drawString("Selection only works while paused. That part is on purpose, not a bug.", 24, 144);
    }

    private void drawSelectedBodyPanel(Graphics2D g2, Body body) {
        double speedSquared = body.vx * body.vx + body.vy * body.vy;
        double speed = Math.sqrt(speedSquared);
        double kineticEnergy = 0.5 * body.mass * speedSquared;
        double potentialEnergy = body.mass * GRAVITY * Math.max(0, getHeight() - body.y);
        double totalEnergy = kineticEnergy + potentialEnergy;

        int panelWidth = 360;
        int panelHeight = 248;
        int x = getWidth() - panelWidth - 16;
        int y = 16;

        g2.setColor(new Color(0, 0, 0, 185));
        g2.fillRoundRect(x, y, panelWidth, panelHeight, 18, 18);
        g2.setColor(Color.WHITE);
        g2.setFont(new Font("Monospaced", Font.PLAIN, 14));

        int lineY = y + 28;
        int step = 18;
        g2.drawString("Selected Body", x + 14, lineY); lineY += step;
        g2.drawString("Shape: " + body.shape.label, x + 14, lineY); lineY += step;
        g2.drawString(String.format("Position: (%.2f, %.2f)", body.x, body.y), x + 14, lineY); lineY += step;
        g2.drawString(String.format("Velocity: (%.2f, %.2f)", body.vx, body.vy), x + 14, lineY); lineY += step;
        g2.drawString(String.format("Acceleration: (%.2f, %.2f)", body.ax, body.ay), x + 14, lineY); lineY += step;
        g2.drawString(String.format("Speed: %.2f", speed), x + 14, lineY); lineY += step;
        g2.drawString(String.format("Mass: %.3f", body.mass), x + 14, lineY); lineY += step;
        g2.drawString(String.format("Width: %.2f | Height: %.2f", body.width, body.height), x + 14, lineY); lineY += step;
        g2.drawString(String.format("Radius: %.2f", body.radius), x + 14, lineY); lineY += step;
        g2.drawString(String.format("Kinetic Energy: %.2f", kineticEnergy), x + 14, lineY); lineY += step;
        g2.drawString(String.format("Potential Energy: %.2f", potentialEnergy), x + 14, lineY); lineY += step;
        g2.drawString(String.format("Total Energy: %.2f", totalEnergy), x + 14, lineY); lineY += step;
        g2.drawString(String.format("AABB: [%.1f, %.1f] to [%.1f, %.1f]", body.getLeft(), body.getTop(), body.getRight(), body.getBottom()), x + 14, lineY);
    }

    private enum SpawnMode {
        CIRCLE("Circle"),
        BOX("Box");

        final String label;

        SpawnMode(String label) {
            this.label = label;
        }
    }

    private static class Body {
        SpawnMode shape;
        double x;
        double y;
        double vx;
        double vy;
        double ax;
        double ay;
        double width;
        double height;
        double radius;
        double mass;
        Color color;

        double halfWidth() {
            return width * 0.5;
        }

        double halfHeight() {
            return height * 0.5;
        }

        double getLeft() {
            return x - halfWidth();
        }

        double getRight() {
            return x + halfWidth();
        }

        double getTop() {
            return y - halfHeight();
        }

        double getBottom() {
            return y + halfHeight();
        }

        boolean contains(double px, double py) {
            if (shape == SpawnMode.CIRCLE) {
                double dx = px - x;
                double dy = py - y;
                return dx * dx + dy * dy <= radius * radius;
            }
            return px >= getLeft() && px <= getRight() && py >= getTop() && py <= getBottom();
        }
    }
}