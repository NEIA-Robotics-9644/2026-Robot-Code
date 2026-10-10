package org.neiacademy.robotics.frc2026.autos;

import static org.junit.jupiter.api.Assertions.*;

import java.io.InputStreamReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.Test;

class NerdBumpGeometryTest {
  private record Point(double x, double y) {
    double distance(Point p) {
      return Math.hypot(x - p.x, y - p.y);
    }
  }

  private static Point point(JSONObject p) {
    return new Point(((Number) p.get("x")).doubleValue(), ((Number) p.get("y")).doubleValue());
  }

  private static List<Point> sample(JSONArray waypoints) {
    var samples = new ArrayList<Point>();
    for (int i = 1; i < waypoints.size(); i++) {
      JSONObject a = (JSONObject) waypoints.get(i - 1), b = (JSONObject) waypoints.get(i);
      Point p0 = point((JSONObject) a.get("anchor")), p1 = point((JSONObject) a.get("nextControl"));
      Point p2 = point((JSONObject) b.get("prevControl")), p3 = point((JSONObject) b.get("anchor"));
      for (int k = 0; k <= 400; k++) {
        double u = k / 400.0, v = 1 - u;
        samples.add(
            new Point(
                v * v * v * p0.x + 3 * v * v * u * p1.x + 3 * v * u * u * p2.x + u * u * u * p3.x,
                v * v * v * p0.y + 3 * v * v * u * p1.y + 3 * v * u * u * p2.y + u * u * u * p3.y));
      }
    }
    return samples;
  }

  private static double length(List<Point> points) {
    double result = 0;
    for (int i = 1; i < points.size(); i++) result += points.get(i).distance(points.get(i - 1));
    return result;
  }

  private static double distanceToSegment(Point p, Point a, Point b) {
    double dx = b.x - a.x, dy = b.y - a.y, square = dx * dx + dy * dy;
    double u =
        square == 0 ? 0 : Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / square));
    return p.distance(new Point(a.x + u * dx, a.y + u * dy));
  }

  private static double deviation(List<Point> a, List<Point> b) {
    double worst = 0;
    for (Point p : a) {
      double nearest = Double.POSITIVE_INFINITY;
      for (int i = 1; i < b.size(); i++)
        nearest = Math.min(nearest, distanceToSegment(p, b.get(i - 1), b.get(i)));
      worst = Math.max(worst, nearest);
    }
    return worst;
  }

  @Test
  void sparseBezierCurvesStayWithinThreePercentOfSourceShapeAndLength() throws Exception {
    JSONObject reference;
    try (var stream = getClass().getResourceAsStream("/nerd-bump-reference.json")) {
      assertNotNull(stream);
      reference = (JSONObject) new JSONParser().parse(new InputStreamReader(stream));
    }
    JSONObject paths = (JSONObject) reference.get("paths");
    assertEquals(7, paths.size());
    for (Object key : paths.keySet()) {
      String name = (String) key;
      JSONObject source = (JSONObject) paths.get(name);
      JSONObject path =
          (JSONObject)
              new JSONParser()
                  .parse(
                      Files.readString(
                          Path.of("src/main/deploy/pathplanner/paths", name + ".path")));
      JSONArray waypoints = (JSONArray) path.get("waypoints");
      assertTrue(
          waypoints.size() <= ((Number) source.get("maxAnchors")).intValue(),
          name + " anchor count");
      assertTrue(((JSONArray) path.get("rotationTargets")).size() <= 12, name + " rotation count");
      List<Point> original = new ArrayList<>();
      for (Object entry : (JSONArray) source.get("points")) {
        JSONArray xy = (JSONArray) entry;
        original.add(
            new Point(((Number) xy.get(0)).doubleValue(), ((Number) xy.get(1)).doubleValue()));
      }
      int first =
          source.containsKey("comparisonStartAnchor")
              ? ((Number) source.get("comparisonStartAnchor")).intValue()
              : 0;
      int last =
          source.containsKey("comparisonAnchorCount")
              ? ((Number) source.get("comparisonAnchorCount")).intValue()
              : waypoints.size();
      JSONArray compared = new JSONArray();
      compared.addAll(waypoints.subList(first, last));
      var fitted = sample(compared);
      if (Boolean.TRUE.equals(source.get("extendedStraight"))) {
        double startX = original.get(0).x;
        assertEquals(6.75, fitted.get(0).x, 1e-9);
        assertEquals(
            3.0,
            ((Number) ((JSONObject) path.get("idealStartingState")).get("velocity")).doubleValue(),
            1e-9);
        assertEquals(
            1.0,
            ((Number)
                    ((JSONObject) ((JSONArray) path.get("rotationTargets")).get(0))
                        .get("waypointRelativePos"))
                .doubleValue(),
            1e-9);
        assertEquals(
            3.03425, point((JSONObject) ((JSONObject) waypoints.get(1)).get("anchor")).x, 1e-9);
        for (Point point : fitted) assertEquals(original.get(0).y, point.y, 1e-9);
        fitted = new ArrayList<>(fitted.stream().filter(point -> point.x < startX).toList());
        fitted.add(0, original.get(0));
      }
      assertEquals(0, original.get(0).distance(fitted.get(0)), 1e-9, name + " start");
      assertEquals(
          0,
          original.get(original.size() - 1).distance(fitted.get(fitted.size() - 1)),
          1e-9,
          name + " finish");
      double minX = original.stream().mapToDouble(Point::x).min().orElseThrow();
      double maxX = original.stream().mapToDouble(Point::x).max().orElseThrow();
      double minY = original.stream().mapToDouble(Point::y).min().orElseThrow();
      double maxY = original.stream().mapToDouble(Point::y).max().orElseThrow();
      // Explicit similarity definition: bidirectional spatial deviation / route bounding diagonal,
      // plus an independent length check. This prevents a shortcut from passing by length alone.
      double scale = Math.hypot(maxX - minX, maxY - minY);
      double error = Math.max(deviation(original, fitted), deviation(fitted, original));
      double lengthError = Math.abs(length(fitted) / length(original) - 1);
      assertTrue(error / scale <= 0.03, name + " shape error " + error / scale);
      assertTrue(lengthError <= 0.03, name + " length error " + lengthError);
      System.out.printf(
          "FIT PASS %s: %d anchors, %.4f m / %.3f%% shape, %.3f%% length%n",
          name, waypoints.size(), error, 100 * error / scale, 100 * lengthError);
    }
  }
}
