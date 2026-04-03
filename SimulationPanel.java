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
import java.awt.event.MouseMotionAdapter;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import javax.swing.JPanel;
import javax.swing.Timer;

public class SimulationPanel extends JPanel {
    private static final double GRAVITY = 900.0;
    private static final double LINEAR_DAMPING = 0.999;
    private static final double ANGULAR_DAMPING = 0.999;
    private static final double RESTITUTION = 0.18;
    private static final double FRICTION = 0.55;
    private static final double WALL_RESTITUTION = 0.16;
    private static final double POSITION_SLOP = 0.01;
    private static final double POSITION_PERCENT = 0.85;
    private static final int TARGET_FPS = 60;
    private static final double FIXED_DT = 1.0 / 120.0;
    private static final int SUBSTEPS = 2;
    private static final int SOLVER_ITERATIONS = 8;
    private static final int INITIAL_BODIES = 14;
    private static final double FORCE_SCALE = 7.0;

    private final List<Body> bodies = new ArrayList<>();
    private final Random random = new Random();

    private Timer timer;
    private long lastTime;
    private double accumulator = 0.0;

    private boolean paused = false;
    private boolean showHUD = true;
    private boolean showVelocityVectors = false;
    private boolean showAccelerationVectors = false;
    private boolean showAABBs = false;

    private SpawnMode currentSpawnMode = SpawnMode.CIRCLE;
    private Body selectedBody = null;

    private Body pausedDraggedBody = null;
    private double pausedDragOffsetX = 0.0;
    private double pausedDragOffsetY = 0.0;

    private Body forceBody = null;
    private Point forceStart = null;
    private Point forceCurrent = null;

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
                        pausedDraggedBody = null;
                        clearForceState();
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
                    Body hit = findBodyAt(e.getPoint());
                    selectedBody = hit;
                    pausedDraggedBody = hit;
                    if (hit != null) {
                        pausedDragOffsetX = hit.x - e.getX();
                        pausedDragOffsetY = hit.y - e.getY();
                        hit.vx = 0.0;
                        hit.vy = 0.0;
                        hit.angularVelocity = 0.0;
                    }
                } else {
                    Body hit = findBodyAt(e.getPoint());
                    if (hit != null) {
                        forceBody = hit;
                        forceStart = e.getPoint();
                        forceCurrent = e.getPoint();
                    } else {
                        spawnBody(e.getX(), e.getY(), currentSpawnMode);
                    }
                }

                repaint();
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                if (paused) {
                    pausedDraggedBody = null;
                } else {
                    if (forceBody != null && forceStart != null) {
                        applyDragForce(forceBody, forceStart, e.getPoint());
                    }
                    clearForceState();
                }
                repaint();
            }
        });

        addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseDragged(MouseEvent e) {
                if (paused) {
                    if (pausedDraggedBody != null) {
                        pausedDraggedBody.x = e.getX() + pausedDragOffsetX;
                        pausedDraggedBody.y = e.getY() + pausedDragOffsetY;
                        pausedDraggedBody.vx = 0.0;
                        pausedDraggedBody.vy = 0.0;
                        pausedDraggedBody.angularVelocity = 0.0;
                        selectedBody = pausedDraggedBody;
                    }
                } else {
                    if (forceBody != null) {
                        forceCurrent = e.getPoint();
                    }
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
        double frameTime = (now - lastTime) / 1_000_000_000.0;
        lastTime = now;
        frameTime = Math.min(frameTime, 0.05);

        if (!paused) {
            accumulator += frameTime;
            while (accumulator >= FIXED_DT) {
                stepSimulation(FIXED_DT);
                accumulator -= FIXED_DT;
            }
        }

        repaint();
    }

    private void stepSimulation(double dt) {
        double subDt = dt / SUBSTEPS;
        for (int s = 0; s < SUBSTEPS; s++) {
            for (Body body : bodies) {
                integrateForces(body, subDt);
            }

            List<Contact> contacts = new ArrayList<>();
            for (int i = 0; i < bodies.size(); i++) {
                for (int j = i + 1; j < bodies.size(); j++) {
                    Contact contact = createContact(bodies.get(i), bodies.get(j));
                    if (contact != null) {
                        contacts.add(contact);
                    }
                }
            }

            for (int i = 0; i < SOLVER_ITERATIONS; i++) {
                for (Contact contact : contacts) {
                    applyImpulse(contact);
                }
            }

            for (Body body : bodies) {
                integrateVelocity(body, subDt);
                handleWallCollision(body);
            }

            for (Contact contact : contacts) {
                positionalCorrection(contact);
            }
        }
    }

    private void integrateForces(Body body, double dt) {
        body.ax = 0.0;
        body.ay = GRAVITY;

        body.vx += body.ax * dt;
        body.vy += body.ay * dt;

        body.vx *= LINEAR_DAMPING;
        body.vy *= LINEAR_DAMPING;
        body.angularVelocity *= ANGULAR_DAMPING;
    }

    private void integrateVelocity(Body body, double dt) {
        body.x += body.vx * dt;
        body.y += body.vy * dt;
        body.angle += body.angularVelocity * dt;
    }

    private void resetSimulation() {
        bodies.clear();
        selectedBody = null;
        pausedDraggedBody = null;
        clearForceState();

        int width = Math.max(getWidth(), 1000);
        int height = Math.max(getHeight(), 700);

        for (int i = 0; i < INITIAL_BODIES; i++) {
            SpawnMode mode = (i % 2 == 0) ? SpawnMode.CIRCLE : SpawnMode.BOX;
            double x = 120 + random.nextDouble() * Math.max(1, width - 240);
            double y = 60 + random.nextDouble() * Math.max(1, height / 2.0 - 120);
            spawnBody(x, y, mode);
        }
    }

    private void spawnBody(double x, double y, SpawnMode mode) {
        Body body = new Body();
        body.shape = mode;
        body.x = x;
        body.y = y;
        body.vx = -140 + random.nextDouble() * 280;
        body.vy = -110 - random.nextDouble() * 110;
        body.ax = 0.0;
        body.ay = GRAVITY;
        body.angle = random.nextDouble() * Math.PI * 2.0;
        body.angularVelocity = -1.8 + random.nextDouble() * 3.6;
        body.color = new Color(
            70 + random.nextInt(160),
            70 + random.nextInt(160),
            70 + random.nextInt(160)
        );

        if (mode == SpawnMode.CIRCLE) {
            body.radius = 14 + random.nextDouble() * 20;
            body.width = body.radius * 2.0;
            body.height = body.radius * 2.0;
            body.mass = Math.PI * body.radius * body.radius * 0.026;
            body.inertia = 0.5 * body.mass * body.radius * body.radius;
        } else {
            body.width = 26 + random.nextDouble() * 30;
            body.height = 26 + random.nextDouble() * 30;
            body.radius = Math.hypot(body.width * 0.5, body.height * 0.5);
            body.mass = body.width * body.height * 0.018;
            body.inertia = body.mass * (body.width * body.width + body.height * body.height) / 12.0;
        }

        body.invMass = body.mass > 0.0 ? 1.0 / body.mass : 0.0;
        body.invInertia = body.inertia > 0.0 ? 1.0 / body.inertia : 0.0;

        bodies.add(body);
    }

    private void applyDragForce(Body body, Point start, Point end) {
        double fx = (start.x - end.x) * FORCE_SCALE;
        double fy = (start.y - end.y) * FORCE_SCALE;
        double contactX = start.x;
        double contactY = start.y;
        applyImpulseAtPoint(body, fx, fy, contactX, contactY);
    }

    private void applyImpulseAtPoint(Body body, double impulseX, double impulseY, double pointX, double pointY) {
        body.vx += impulseX * body.invMass;
        body.vy += impulseY * body.invMass;

        double rx = pointX - body.x;
        double ry = pointY - body.y;
        double torqueImpulse = cross(rx, ry, impulseX, impulseY);
        body.angularVelocity += torqueImpulse * body.invInertia;
    }

    private void clearForceState() {
        forceBody = null;
        forceStart = null;
        forceCurrent = null;
    }

    private void handleWallCollision(Body body) {
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;

        for (Vec2 v : body.getWorldVertices()) {
            minX = Math.min(minX, v.x);
            maxX = Math.max(maxX, v.x);
            minY = Math.min(minY, v.y);
            maxY = Math.max(maxY, v.y);
        }

        if (minX < 0.0) {
            body.x += -minX;
            body.vx = Math.abs(body.vx) * WALL_RESTITUTION;
            body.angularVelocity *= 0.92;
        }
        if (maxX > getWidth()) {
            body.x -= (maxX - getWidth());
            body.vx = -Math.abs(body.vx) * WALL_RESTITUTION;
            body.angularVelocity *= 0.92;
        }
        if (minY < 0.0) {
            body.y += -minY;
            body.vy = Math.abs(body.vy) * WALL_RESTITUTION;
            body.angularVelocity *= 0.92;
        }
        if (maxY > getHeight()) {
            body.y -= (maxY - getHeight());
            body.vy = -Math.abs(body.vy) * WALL_RESTITUTION;
            body.vx *= 0.965;
            body.angularVelocity *= 0.95;
            if (Math.abs(body.vy) < 9.0) {
                body.vy = 0.0;
            }
        }
    }

    private Contact createContact(Body a, Body b) {
        if (a.shape == SpawnMode.CIRCLE && b.shape == SpawnMode.CIRCLE) {
            return circleCircleContact(a, b);
        }
        if (a.shape == SpawnMode.BOX && b.shape == SpawnMode.BOX) {
            return boxBoxContact(a, b);
        }
        if (a.shape == SpawnMode.BOX && b.shape == SpawnMode.CIRCLE) {
            return boxCircleContact(a, b);
        }
        Contact c = boxCircleContact(b, a);
        if (c == null) {
            return null;
        }
        return new Contact(a, b, -c.normalX, -c.normalY, c.penetration, c.contactX, c.contactY);
    }

    private Contact circleCircleContact(Body a, Body b) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double distSq = dx * dx + dy * dy;
        double r = a.radius + b.radius;

        if (distSq >= r * r) {
            return null;
        }

        double dist = Math.sqrt(Math.max(distSq, 1e-12));
        double nx;
        double ny;
        if (dist < 1e-6) {
            nx = 1.0;
            ny = 0.0;
            dist = r - 0.001;
        } else {
            nx = dx / dist;
            ny = dy / dist;
        }

        double penetration = r - dist;
        double contactX = a.x + nx * a.radius;
        double contactY = a.y + ny * a.radius;
        return new Contact(a, b, nx, ny, penetration, contactX, contactY);
    }

    private Contact boxCircleContact(Body box, Body circle) {
        double cos = Math.cos(-box.angle);
        double sin = Math.sin(-box.angle);

        double relX = circle.x - box.x;
        double relY = circle.y - box.y;
        double localX = relX * cos - relY * sin;
        double localY = relX * sin + relY * cos;

        double halfW = box.width * 0.5;
        double halfH = box.height * 0.5;

        double closestX = clamp(localX, -halfW, halfW);
        double closestY = clamp(localY, -halfH, halfH);

        double deltaX = localX - closestX;
        double deltaY = localY - closestY;
        double distSq = deltaX * deltaX + deltaY * deltaY;

        if (distSq > circle.radius * circle.radius) {
            return null;
        }

        double localNormalX;
        double localNormalY;
        double penetration;
        double contactLocalX = closestX;
        double contactLocalY = closestY;

        if (distSq > 1e-10) {
            double dist = Math.sqrt(distSq);
            localNormalX = deltaX / dist;
            localNormalY = deltaY / dist;
            penetration = circle.radius - dist;
        } else {
            double dxLeft = Math.abs(localX + halfW);
            double dxRight = Math.abs(halfW - localX);
            double dyTop = Math.abs(localY + halfH);
            double dyBottom = Math.abs(halfH - localY);
            double min = dxLeft;
            localNormalX = -1.0;
            localNormalY = 0.0;
            contactLocalX = -halfW;
            contactLocalY = localY;

            if (dxRight < min) {
                min = dxRight;
                localNormalX = 1.0;
                localNormalY = 0.0;
                contactLocalX = halfW;
                contactLocalY = localY;
            }
            if (dyTop < min) {
                min = dyTop;
                localNormalX = 0.0;
                localNormalY = -1.0;
                contactLocalX = localX;
                contactLocalY = -halfH;
            }
            if (dyBottom < min) {
                min = dyBottom;
                localNormalX = 0.0;
                localNormalY = 1.0;
                contactLocalX = localX;
                contactLocalY = halfH;
            }
            penetration = circle.radius + min;
        }

        double worldNormalX = localNormalX * Math.cos(box.angle) - localNormalY * Math.sin(box.angle);
        double worldNormalY = localNormalX * Math.sin(box.angle) + localNormalY * Math.cos(box.angle);
        double contactX = box.x + contactLocalX * Math.cos(box.angle) - contactLocalY * Math.sin(box.angle);
        double contactY = box.y + contactLocalX * Math.sin(box.angle) + contactLocalY * Math.cos(box.angle);

        return new Contact(box, circle, worldNormalX, worldNormalY, penetration, contactX, contactY);
    }

    private Contact boxBoxContact(Body a, Body b) {
        Vec2[] axes = new Vec2[4];
        Vec2[] aAxes = a.getAxes();
        Vec2[] bAxes = b.getAxes();
        axes[0] = aAxes[0];
        axes[1] = aAxes[1];
        axes[2] = bAxes[0];
        axes[3] = bAxes[1];

        double smallestOverlap = Double.POSITIVE_INFINITY;
        Vec2 smallestAxis = null;

        Vec2[] aVerts = a.getWorldVertices();
        Vec2[] bVerts = b.getWorldVertices();

        for (Vec2 axis : axes) {
            double[] projA = projectVertices(aVerts, axis);
            double[] projB = projectVertices(bVerts, axis);
            double overlap = Math.min(projA[1], projB[1]) - Math.max(projA[0], projB[0]);
            if (overlap <= 0.0) {
                return null;
            }
            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                smallestAxis = axis;
            }
        }

        if (smallestAxis == null) {
            return null;
        }

        double dirX = b.x - a.x;
        double dirY = b.y - a.y;
        if (dot(dirX, dirY, smallestAxis.x, smallestAxis.y) < 0.0) {
            smallestAxis = new Vec2(-smallestAxis.x, -smallestAxis.y);
        }

        double contactX = (a.x + b.x) * 0.5;
        double contactY = (a.y + b.y) * 0.5;
        return new Contact(a, b, smallestAxis.x, smallestAxis.y, smallestOverlap, contactX, contactY);
    }

    private void applyImpulse(Contact contact) {
        Body a = contact.a;
        Body b = contact.b;

        double rxA = contact.contactX - a.x;
        double ryA = contact.contactY - a.y;
        double rxB = contact.contactX - b.x;
        double ryB = contact.contactY - b.y;

        double velAX = a.vx + (-a.angularVelocity * ryA);
        double velAY = a.vy + (a.angularVelocity * rxA);
        double velBX = b.vx + (-b.angularVelocity * ryB);
        double velBY = b.vy + (b.angularVelocity * rxB);

        double rvx = velBX - velAX;
        double rvy = velBY - velAY;

        double velAlongNormal = rvx * contact.normalX + rvy * contact.normalY;
        if (velAlongNormal > 0.0) {
            return;
        }

        double raCrossN = cross(rxA, ryA, contact.normalX, contact.normalY);
        double rbCrossN = cross(rxB, ryB, contact.normalX, contact.normalY);
        double invMassSum = a.invMass + b.invMass + (raCrossN * raCrossN) * a.invInertia + (rbCrossN * rbCrossN) * b.invInertia;
        if (invMassSum <= 0.0) {
            return;
        }

        double normalImpulseMag = -(1.0 + RESTITUTION) * velAlongNormal / invMassSum;
        double impulseX = normalImpulseMag * contact.normalX;
        double impulseY = normalImpulseMag * contact.normalY;

        a.vx -= impulseX * a.invMass;
        a.vy -= impulseY * a.invMass;
        a.angularVelocity -= cross(rxA, ryA, impulseX, impulseY) * a.invInertia;

        b.vx += impulseX * b.invMass;
        b.vy += impulseY * b.invMass;
        b.angularVelocity += cross(rxB, ryB, impulseX, impulseY) * b.invInertia;

        double tangentX = rvx - velAlongNormal * contact.normalX;
        double tangentY = rvy - velAlongNormal * contact.normalY;
        double tangentLength = Math.hypot(tangentX, tangentY);
        if (tangentLength < 1e-8) {
            return;
        }
        tangentX /= tangentLength;
        tangentY /= tangentLength;

        double raCrossT = cross(rxA, ryA, tangentX, tangentY);
        double rbCrossT = cross(rxB, ryB, tangentX, tangentY);
        double tangentMassSum = a.invMass + b.invMass + (raCrossT * raCrossT) * a.invInertia + (rbCrossT * rbCrossT) * b.invInertia;
        if (tangentMassSum <= 0.0) {
            return;
        }

        double jt = -(rvx * tangentX + rvy * tangentY) / tangentMassSum;
        double maxFriction = normalImpulseMag * FRICTION;
        jt = clamp(jt, -maxFriction, maxFriction);

        double frictionX = jt * tangentX;
        double frictionY = jt * tangentY;

        a.vx -= frictionX * a.invMass;
        a.vy -= frictionY * a.invMass;
        a.angularVelocity -= cross(rxA, ryA, frictionX, frictionY) * a.invInertia;

        b.vx += frictionX * b.invMass;
        b.vy += frictionY * b.invMass;
        b.angularVelocity += cross(rxB, ryB, frictionX, frictionY) * b.invInertia;
    }

    private void positionalCorrection(Contact contact) {
        double invMassSum = contact.a.invMass + contact.b.invMass;
        if (invMassSum <= 0.0) {
            return;
        }

        double correctionMag = Math.max(contact.penetration - POSITION_SLOP, 0.0) / invMassSum * POSITION_PERCENT;
        double correctionX = correctionMag * contact.normalX;
        double correctionY = correctionMag * contact.normalY;

        contact.a.x -= correctionX * contact.a.invMass;
        contact.a.y -= correctionY * contact.a.invMass;
        contact.b.x += correctionX * contact.b.invMass;
        contact.b.y += correctionY * contact.b.invMass;
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
        drawForcePreview(g2);

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
                g2.setColor(new Color(255, 255, 255, 35));
                g2.fillRect((int) Math.round(body.getAABBMinX() - 8), (int) Math.round(body.getAABBMinY() - 8), (int) Math.round(body.getAABBMaxX() - body.getAABBMinX() + 16), (int) Math.round(body.getAABBMaxY() - body.getAABBMinY() + 16));
            }

            g2.setColor(body.color);
            if (body.shape == SpawnMode.CIRCLE) {
                int left = (int) Math.round(body.x - body.radius);
                int top = (int) Math.round(body.y - body.radius);
                int diameter = (int) Math.round(body.radius * 2.0);
                g2.fillOval(left, top, diameter, diameter);
                g2.setColor(new Color(255, 255, 255, 90));
                g2.fillOval((int) Math.round(body.x - body.radius * 0.35), (int) Math.round(body.y - body.radius * 0.5), Math.max(4, (int) Math.round(body.radius * 0.5)), Math.max(4, (int) Math.round(body.radius * 0.5)));
                g2.setColor(Color.BLACK);
                g2.drawOval(left, top, diameter, diameter);
                g2.drawLine((int) Math.round(body.x), (int) Math.round(body.y), (int) Math.round(body.x + Math.cos(body.angle) * body.radius), (int) Math.round(body.y + Math.sin(body.angle) * body.radius));
            } else {
                Vec2[] vertices = body.getWorldVertices();
                Path2D.Double path = new Path2D.Double();
                path.moveTo(vertices[0].x, vertices[0].y);
                for (int i = 1; i < vertices.length; i++) {
                    path.lineTo(vertices[i].x, vertices[i].y);
                }
                path.closePath();
                g2.fill(path);
                g2.setColor(Color.BLACK);
                g2.draw(path);
                g2.drawLine((int) Math.round(body.x), (int) Math.round(body.y), (int) Math.round(body.x + Math.cos(body.angle) * body.width * 0.5), (int) Math.round(body.y + Math.sin(body.angle) * body.width * 0.5));
            }
        }
    }

    private void drawDebugOverlays(Graphics2D g2) {
        for (Body body : bodies) {
            if (showAABBs) {
                g2.setColor(new Color(255, 80, 80, 180));
                g2.drawRect((int) Math.round(body.getAABBMinX()), (int) Math.round(body.getAABBMinY()), (int) Math.round(body.getAABBMaxX() - body.getAABBMinX()), (int) Math.round(body.getAABBMaxY() - body.getAABBMinY()));
            }

            if (showVelocityVectors) {
                drawVector(g2, body.x, body.y, body.vx * 0.20, body.vy * 0.20, new Color(80, 220, 255));
            }

            if (showAccelerationVectors) {
                drawVector(g2, body.x, body.y, body.ax * 0.05, body.ay * 0.05, new Color(255, 210, 80));
            }
        }
    }

    private void drawForcePreview(Graphics2D g2) {
        if (forceBody == null || forceStart == null || forceCurrent == null || paused) {
            return;
        }
        drawVector(g2, forceStart.x, forceStart.y, forceStart.x - forceCurrent.x, forceStart.y - forceCurrent.y, new Color(150, 255, 150));
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
        g2.fillRoundRect(12, 12, 860, 160, 16, 16);

        g2.setColor(Color.WHITE);
        g2.drawString("Advanced 2D Physics Simulator", 24, 34);
        g2.drawString("Bodies: " + bodies.size() + " | Status: " + (paused ? "PAUSED" : "RUNNING") + " | Spawn: " + currentSpawnMode.label, 24, 56);
        g2.drawString("1 = circle | 2 = box | B = spawn center | V = velocity | A = acceleration | X = AABB", 24, 78);
        g2.drawString("Space = pause | R = reset | C = clear | H = hide HUD", 24, 100);
        g2.drawString("Paused: click and drag a body to reposition it, click to inspect it", 24, 122);
        g2.drawString("Running: click empty space to spawn, click-drag on a body to apply force and torque", 24, 144);
        g2.drawString("Fixed timestep, substeps, angular impulse, box-circle collision, and less jittery stacking are now in.", 24, 166);
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
        g2.drawString(String.format("AABB: [%.1f, %.1f] to [%.1f, %.1f]", body.getAABBMinX(), body.getAABBMinY(), body.getAABBMaxX(), body.getAABBMaxY()), x + 14, lineY);
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    private static double dot(double ax, double ay, double bx, double by) {
        return ax * bx + ay * by;
    }
    private static double cross(double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    }
    private static double[] projectVertices(Vec2[] verts, Vec2 axis) {
        double min = dot(verts[0].x, verts[0].y, axis.x, axis.y);
        double max = min;
        for (int i = 1; i < verts.length; i++) {
            double p = dot(verts[i].x, verts[i].y, axis.x, axis.y);
            min = Math.min(min, p);
            max = Math.max(max, p);
        }
        return new double[]{min, max};
    }

    private enum SpawnMode {
        CIRCLE("Circle"),
        BOX("Box");
        final String label;
        SpawnMode(String label) { this.label = label; }
    }

    private static class Vec2 {
        final double x, y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
    }

    private static class Body {
        SpawnMode shape;
        double x, y;
        double vx, vy;
        double ax, ay;
        double angle, angularVelocity;
        double width, height, radius;
        double mass, inertia;
        double invMass, invInertia;
        Color color;

        boolean contains(double px, double py) {
            if (shape == SpawnMode.CIRCLE) {
                double dx = px - x;
                double dy = py - y;
                return dx * dx + dy * dy <= radius * radius;
            }
            // Box: transform point into local box coordinates
            double cos = Math.cos(-angle);
            double sin = Math.sin(-angle);
            double relX = px - x;
            double relY = py - y;
            double localX = relX * cos - relY * sin;
            double localY = relX * sin + relY * cos;
            return Math.abs(localX) <= width * 0.5 && Math.abs(localY) <= height * 0.5;
        }

        Vec2[] getWorldVertices() {
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            double hw = width * 0.5;
            double hh = height * 0.5;
            Vec2[] verts = new Vec2[4];
            verts[0] = new Vec2(x + (-hw) * cos - (-hh) * sin, y + (-hw) * sin + (-hh) * cos);
            verts[1] = new Vec2(x + (hw) * cos - (-hh) * sin, y + (hw) * sin + (-hh) * cos);
            verts[2] = new Vec2(x + (hw) * cos - (hh) * sin, y + (hw) * sin + (hh) * cos);
            verts[3] = new Vec2(x + (-hw) * cos - (hh) * sin, y + (-hw) * sin + (hh) * cos);
            return verts;
        }
        Vec2[] getAxes() {
            Vec2[] verts = getWorldVertices();
            Vec2[] axes = new Vec2[2];
            axes[0] = new Vec2(verts[1].x - verts[0].x, verts[1].y - verts[0].y);
            axes[1] = new Vec2(verts[3].x - verts[0].x, verts[3].y - verts[0].y);
            double len0 = Math.hypot(axes[0].x, axes[0].y);
            double len1 = Math.hypot(axes[1].x, axes[1].y);
            axes[0] = new Vec2(axes[0].x / len0, axes[0].y / len0);
            axes[1] = new Vec2(axes[1].x / len1, axes[1].y / len1);
            return axes;
        }
        double getAABBMinX() {
            Vec2[] verts = getWorldVertices();
            double min = verts[0].x;
            for (int i = 1; i < verts.length; i++) min = Math.min(min, verts[i].x);
            return min;
        }
        double getAABBMaxX() {
            Vec2[] verts = getWorldVertices();
            double max = verts[0].x;
            for (int i = 1; i < verts.length; i++) max = Math.max(max, verts[i].x);
            return max;
        }
        double getAABBMinY() {
            Vec2[] verts = getWorldVertices();
            double min = verts[0].y;
            for (int i = 1; i < verts.length; i++) min = Math.min(min, verts[i].y);
            return min;
        }
        double getAABBMaxY() {
            Vec2[] verts = getWorldVertices();
            double max = verts[0].y;
            for (int i = 1; i < verts.length; i++) max = Math.max(max, verts[i].y);
            return max;
        }
    }

    private static class Contact {
        final Body a, b;
        final double normalX, normalY;
        final double penetration;
        final double contactX, contactY;
        Contact(Body a, Body b, double nx, double ny, double penetration, double cx, double cy) {
            this.a = a; this.b = b;
            this.normalX = nx; this.normalY = ny;
            this.penetration = penetration;
            this.contactX = cx; this.contactY = cy;
        }
    }
}