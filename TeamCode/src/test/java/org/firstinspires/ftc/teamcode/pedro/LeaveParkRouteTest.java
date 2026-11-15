package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.junit.Test;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.pedro.LeaveParkRoute.Alliance;

public class LeaveParkRouteTest {
    private static final double EPS = 1e-8;
    private static final double FIELD = 141.5;

    private double[] bounds(Pose pose, double length, double width) {
        double xRadius = Math.abs(Math.cos(pose.heading())) * length / 2
                + Math.abs(Math.sin(pose.heading())) * width / 2;
        double yRadius = Math.abs(Math.sin(pose.heading())) * length / 2
                + Math.abs(Math.cos(pose.heading())) * width / 2;
        return new double[] {pose.x() - xRadius, pose.y() - yRadius,
                pose.x() + xRadius, pose.y() + yRadius};
    }

    private void assertInBounds(double[] b) {
        assertTrue("left edge", b[0] >= -EPS);
        assertTrue("bottom edge", b[1] >= -EPS);
        assertTrue("right edge", b[2] <= FIELD + EPS);
        assertTrue("top edge", b[3] <= FIELD + EPS);
    }

    private double[] zone(Alliance alliance) {
        return alliance == Alliance.RED ? new double[] {0, 94.5, 11, 118.5}
                : new double[] {130.5, 23, 141.5, 47};
    }

    private boolean overlaps(double[] a, double[] b) {
        return Math.min(a[2], b[2]) > Math.max(a[0], b[0])
                && Math.min(a[3], b[3]) > Math.max(a[1], b[1]);
    }

    @Test public void matchesTheReviewedVisualizerPoints() {
        double[][] red = {{9,129.5},{20.62,128.42},{19.15,115.18},{15,114.16}};
        double[][] blue = {{132.5,12},{120.88,13.08},{122.35,26.32},{126.5,27.34}};
        for (Alliance alliance : Alliance.values()) {
            Pose[] poses = LeaveParkRoute.poses(alliance);
            double[][] expected = alliance == Alliance.RED ? red : blue;
            for (int i = 0; i < poses.length; i++) {
                assertEquals(expected[i][0], poses[i].x(), EPS);
                assertEquals(expected[i][1], poses[i].y(), EPS);
                assertEquals(alliance == Alliance.RED ? 0 : Math.PI, poses[i].heading(), EPS);
            }
        }
    }

    @Test public void blueIsAnExactHalfTurnOfRed() {
        Pose[] red = LeaveParkRoute.poses(Alliance.RED);
        Pose[] blue = LeaveParkRoute.poses(Alliance.BLUE);
        for (int i = 0; i < red.length; i++) {
            assertEquals(FIELD - red[i].x(), blue[i].x(), EPS);
            assertEquals(FIELD - red[i].y(), blue[i].y(), EPS);
            assertEquals(Math.PI, blue[i].heading() - red[i].heading(), EPS);
        }
    }

    @Test public void startsTouchingExactlyOneWallOutsideLoadingZones() {
        for (Alliance alliance : Alliance.values()) {
            Pose start = LeaveParkRoute.poses(alliance)[0];
            double[] b = bounds(start, 18, 18);
            assertInBounds(b);
            assertEquals(0, alliance == Alliance.RED ? b[0] : FIELD - b[2], EPS);
            assertTrue(b[1] > 0 && b[3] < FIELD);
            assertFalse(overlaps(b, zone(alliance)));
        }
    }

    @Test public void parksWithSixInchWallGapAndFiveInchZoneOverlap() {
        for (Alliance alliance : Alliance.values()) {
            Pose park = LeaveParkRoute.poses(alliance)[3];
            double[] b = bounds(park, 18, 18);
            assertInBounds(b);
            double gap = alliance == Alliance.RED ? b[0] : FIELD - b[2];
            assertEquals(6, gap, EPS);
            assertTrue(overlaps(b, zone(alliance)));
            assertEquals(5, 11 - gap, EPS);
        }
    }

    @Test public void entirePathsKeepFootprintsInBoundsOnOwnAllianceSide() {
        for (Alliance alliance : Alliance.values()) {
            for (Path path : LeaveParkRoute.paths(alliance)) {
                for (int i = 0; i <= 100; i++) {
                    double[] b = bounds(path.get(i / 100.0), 18, 18);
                    assertInBounds(b);
                    assertTrue(alliance == Alliance.RED ? b[2] < FIELD / 2 : b[0] > FIELD / 2);
                }
            }
        }
    }

    @Test public void parkingStillQualifiesAtAllowedPositionAndHeadingErrors() {
        for (Alliance alliance : Alliance.values()) {
            Pose target = LeaveParkRoute.poses(alliance)[3];
            for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++) {
                for (int dh = -1; dh <= 1; dh++) {
                    Pose actual = new Pose(target.x() + dx * 1.5, target.y() + dy * 1.5,
                            target.heading() + dh * Math.toRadians(5));
                    double[] b = bounds(actual, 18, 18);
                    assertInBounds(b);
                    assertTrue("must clear all walls", b[0] > 0 && b[1] > 0 && b[2] < FIELD && b[3] < FIELD);
                    assertTrue("must still overlap loading zone", overlaps(b, zone(alliance)));
                }
            }
        }
    }

    @Test public void smallerFootprintsKeepTheSameWallGap() {
        for (int length = 8; length <= 18; length++) for (int width = 8; width <= 18; width++) {
            for (Alliance alliance : Alliance.values()) {
                Pose[] p = LeaveParkRoute.poses(alliance, length, width);
                double[] start = bounds(p[0], length, width), park = bounds(p[3], length, width);
                assertInBounds(start);
                assertInBounds(park);
                assertEquals(0, alliance == Alliance.RED ? start[0] : FIELD - start[2], EPS);
                assertEquals(6, alliance == Alliance.RED ? park[0] : FIELD - park[2], EPS);
                assertFalse(overlaps(start, zone(alliance)));
                assertTrue(overlaps(park, zone(alliance)));
            }
        }
    }

    @Test public void buildsConnectedPathsWithNoHeadingJumps() {
        for (Alliance alliance : Alliance.values()) {
            Path[] paths = LeaveParkRoute.paths(alliance);
            assertEquals(3, paths.length);
            for (int i = 0; i < paths.length; i++) {
                assertEquals(alliance == Alliance.RED ? 0 : Math.PI, paths[i].heading(0.5), EPS);
                if (i > 0) {
                    assertEquals(0, paths[i - 1].endPose().distance(paths[i].get(0)), EPS);
                    assertEquals(paths[i - 1].heading(1), paths[i].heading(0), EPS);
                }
            }
        }
    }

    @Test(expected = IllegalArgumentException.class) public void rejectsOversizedFootprint() {
        LeaveParkRoute.poses(Alliance.RED, 19, 18);
    }
}
