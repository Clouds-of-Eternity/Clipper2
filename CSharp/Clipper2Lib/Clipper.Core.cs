/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  12 December 2025                                                *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2025                                         *
* Purpose   :  Core structures and functions for the Clipper Library           *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#nullable enable
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using EternityWorks;

namespace Clipper2Lib
{
  public class PathPoint : List<Point> 
  {
    public PathPoint() : base() { }
    public PathPoint(int capacity = 0) : base(capacity) { }
    public PathPoint(IEnumerable<Point> path) : base(path) { }
    public override string ToString()
    {
      return string.Join(", ", this);
    }
  }

  public class PathsPoint : List<PathPoint>
  {
    public PathsPoint() : base() { }
    public PathsPoint(int capacity = 0) : base(capacity) { }
    public PathsPoint(IEnumerable<PathPoint> paths) : base(paths) { }
    public override string ToString()
    {
      return string.Join(Environment.NewLine, this);
    }
  }

  public class PathVector2 : List<Vector2>
  {
    public PathVector2() : base() { }
    public PathVector2(int capacity = 0) : base(capacity) { }
    public PathVector2(IEnumerable<Vector2> path) : base(path) { }
    public override string ToString()
    {
      return string.Join(", ", ConvertAll(x => x.ToString()));
    }
  }

  public class PathsVector2 : List<PathVector2>
  {
    public PathsVector2() : base() { }
    public PathsVector2(int capacity = 0) : base(capacity) { }
    public PathsVector2(IEnumerable<PathVector2> paths) : base(paths) { }
    public override string ToString()
    {
      return string.Join(Environment.NewLine, ConvertAll(x => x.ToString()));
    }
  }

  // Note: all clipping operations except for Difference are commutative.
  public enum ClipType
  {
    NoClip,
    Intersection,
    Union,
    Difference,
    Xor
  }

  public enum PathType
  {
    Subject,
    Clip
  }

  // By far the most widely used filling rules for polygons are EvenOdd
  // and NonZero, sometimes called Alternate and Winding respectively.
  // https://en.wikipedia.org/wiki/Nonzero-rule
  public enum FillRule
  {
    EvenOdd,
    NonZero,
    Positive,
    Negative
  }

  public static class InternalClipper
  {
    internal const int MaxInt64 = int.MaxValue;
    internal const int MaxCoord = MaxInt64 / 4;
    internal const float max_coord = MaxCoord;
    internal const float min_coord = -MaxCoord;
    internal const int Invalid64 = MaxInt64;

    internal const float floatingPointTolerance = 1E-12f;
    internal const float defaultMinimumEdgeLength = 0.1f;

    private static readonly string
      precision_range_error = "Error: Precision is out of range.";

    public static float CrossProduct(Point pt1, Point pt2, Point pt3)
    {
      // typecast to float to avoid potential int overflow
      return ((float) (pt2.X - pt1.X) * (pt3.Y - pt2.Y) -
              (float) (pt2.Y - pt1.Y) * (pt3.X - pt2.X));
    }

    public static int CrossProductSign(Point pt1, Point pt2, Point pt3)
    {
      int a = pt2.X - pt1.X;
      int b = pt3.Y - pt2.Y;
      int c = pt2.Y - pt1.Y;
      int d = pt3.X - pt2.X;
      UInt128Struct ab = MultiplyUInt64((uint) Math.Abs(a), (uint) Math.Abs(b));
      UInt128Struct cd = MultiplyUInt64((uint) Math.Abs(c), (uint) Math.Abs(d));
      int signAB = TriSign(a) * TriSign(b);
      int signCD = TriSign(c) * TriSign(d);

      if (signAB == signCD)
      {
        int result;
        if (ab.hi64 == cd.hi64)
        {
          if (ab.lo64 == cd.lo64) return 0;
          result = (ab.lo64 > cd.lo64) ? 1 : -1;
        }
        else result = (ab.hi64 > cd.hi64) ? 1 : -1;
        return (signAB > 0) ? result : -result;
      }
      return (signAB > signCD) ? 1 : -1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void CheckPrecision(int precision)
    {
      if (precision < -8 || precision > 8)
        throw new Exception(precision_range_error);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsAlmostZero(float value)
    {
      return (Math.Abs(value) <= floatingPointTolerance);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int TriSign(int x) // returns 0, 1 or -1
    {
      return (x < 0) ? -1 : (x > 0) ? 1 : 0;
    }

    public struct UInt128Struct
    {
      public uint lo64;
      public uint hi64;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static UInt128Struct MultiplyUInt64(uint a, uint b) // #834,#835
    {
      uint x1 = (a & 0xFFFFFFFF) * (b & 0xFFFFFFFF);
      uint x2 = (a >> 32) * (b & 0xFFFFFFFF) + (x1 >> 32);
      uint x3 = (a & 0xFFFFFFFF) * (b >> 32) + (x2 & 0xFFFFFFFF);
      UInt128Struct result; 
      result.lo64 = (x3 & 0xFFFFFFFF) << 32 | (x1 & 0xFFFFFFFF);
      result.hi64 = (a >> 32) * (b >> 32) + (x2 >> 32) + (x3 >> 32);
      return result;
    }

    // returns true if (and only if) a * b == c * d
    internal static bool ProductsAreEqual(int a, int b, int c, int d)
    {
      // nb: unsigned values will be needed for CalcOverflowCarry()
      uint absA = (uint) Math.Abs(a);
      uint absB = (uint) Math.Abs(b);
      uint absC = (uint) Math.Abs(c);
      uint absD = (uint) Math.Abs(d);

      UInt128Struct mul_ab = MultiplyUInt64(absA, absB);
      UInt128Struct mul_cd = MultiplyUInt64(absC, absD);

      // nb: it's important to differentiate 0 values here from other values
      int sign_ab = TriSign(a) * TriSign(b);
      int sign_cd = TriSign(c) * TriSign(d);

      return mul_ab.lo64 == mul_cd.lo64 && mul_ab.hi64 == mul_cd.hi64 && sign_ab == sign_cd;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsCollinear(Point pt1, Point sharedPt, Point pt2)
    {
      int a = sharedPt.X - pt1.X;
      int b = pt2.Y - sharedPt.Y;
      int c = sharedPt.Y - pt1.Y;
      int d = pt2.X - sharedPt.X;
      // When checking for collinearity with very large coordinate values
      // then ProductsAreEqual is more accurate than using CrossProduct.
      return ProductsAreEqual(a, b, c, d);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float DotProduct(Point pt1, Point pt2, Point pt3)
    {
      // typecast to float to avoid potential int overflow
      return ((float) (pt2.X - pt1.X) * (pt3.X - pt2.X) +
              (float) (pt2.Y - pt1.Y) * (pt3.Y - pt2.Y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float CrossProduct(Vector2 vec1, Vector2 vec2)
    {
      return (vec1.Y * vec2.X - vec2.Y * vec1.X);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float DotProduct(Vector2 vec1, Vector2 vec2)
    {
      return (vec1.X * vec2.X + vec1.Y * vec2.Y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int CheckCastInt64(float val)
    {
      if ((val >= max_coord) || (val <= min_coord)) return Invalid64;
      return (int)Math.Round(val, MidpointRounding.AwayFromZero);
    }

    // GetLineIntersectPt - a 'true' result is non-parallel. The 'ip' will also
    // be constrained to seg1. However, it's possible that 'ip' won't be inside
    // seg2, even when 'ip' hasn't been constrained (ie 'ip' is inside seg1).

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool GetLineIntersectPt(Point ln1a,
      Point ln1b, Point ln2a, Point ln2b, out Point ip)
    {
      float dy1 = (ln1b.Y - ln1a.Y);
      float dx1 = (ln1b.X - ln1a.X);
      float dy2 = (ln2b.Y - ln2a.Y);
      float dx2 = (ln2b.X - ln2a.X);
      float det = dy1 * dx2 - dy2 * dx1;
      if (det == 0.0)
      {
        ip = new Point();
        return false;
      }

      float t = ((ln1a.X - ln2a.X) * dy2 - (ln1a.Y - ln2a.Y) * dx2) / det;
      if (t <= 0.0) ip = ln1a;
      else if (t >= 1.0) ip = ln1b;
      else
      {
        // avoid using constructor (and rounding too) as they affect performance //664
        ip.X = (int) (ln1a.X + t * dx1);
        ip.Y = (int) (ln1a.Y + t * dy1);
      }
      return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool GetLineIntersectPt(Vector2 ln1a,
      Vector2 ln1b, Vector2 ln2a, Vector2 ln2b, out Vector2 ip)
    {
      float dy1 = (ln1b.Y - ln1a.Y);
      float dx1 = (ln1b.X - ln1a.X);
      float dy2 = (ln2b.Y - ln2a.Y);
      float dx2 = (ln2b.X - ln2a.X);
      float det = dy1 * dx2 - dy2 * dx1;
      if (det == 0.0)
      {
        ip = new Vector2();
        return false;
      }

      float t = ((ln1a.X - ln2a.X) * dy2 - (ln1a.Y - ln2a.Y) * dx2) / det;
      if (t <= 0.0) ip = ln1a;
      else if (t >= 1.0) ip = ln1b;
      else
      {
        // avoid using constructor (and rounding too) as they affect performance //664
        ip.X = (ln1a.X + t * dx1);
        ip.Y = (ln1a.Y + t * dy1);
      }
      return true;
    }

    internal static bool SegsIntersect(Point seg1a, 
      Point seg1b, Point seg2a, Point seg2b, bool inclusive = false)
    {
      float dy1 = (seg1b.Y - seg1a.Y);
      float dx1 = (seg1b.X - seg1a.X);
      float dy2 = (seg2b.Y - seg2a.Y);
      float dx2 = (seg2b.X - seg2a.X);
      float cp = dy1 * dx2 - dy2 * dx1;
      if (cp == 0) return false; // ie parallel segments

      if (inclusive)
      {
        //result **includes** segments that touch at an end point
        float t = ((seg1a.X - seg2a.X) * dy2 - (seg1a.Y - seg2a.Y) * dx2);
        if (t == 0) return true;
        if (t > 0)
        {
          if (cp < 0 || t > cp) return false;
        }
        else if (cp > 0 || t < cp) return false; // false when t more neg. than cp

        t = ((seg1a.X - seg2a.X) * dy1 - (seg1a.Y - seg2a.Y) * dx1);
        if (t == 0) return true;
        if (t > 0) return (cp > 0 && t <= cp);
        else return (cp < 0 && t >= cp);        // true when t less neg. than cp
      }
      else
      {
        //result **excludes** segments that touch at an end point
        float t = ((seg1a.X - seg2a.X) * dy2 - (seg1a.Y - seg2a.Y) * dx2);
        if (t == 0) return false;
        if (t > 0)
        {
          if (cp < 0 || t >= cp) return false;
        }
        else if (cp > 0 || t <= cp) return false; // false when t more neg. than cp

        t = ((seg1a.X - seg2a.X) * dy1 - (seg1a.Y - seg2a.Y) * dx1);
        if (t == 0) return false;
        if (t > 0) return (cp > 0 && t < cp);
        else return (cp < 0 && t > cp); // true when t less neg. than cp
      }
    }

    public static Rectangle GetBounds(PathPoint path)
    {
      if (path.Count == 0) return new Rectangle();
      Rectangle result = Clipper.InvalidRectangle;
      foreach (Point pt in path)
      {
        if (pt.X < result.Left) result.X = pt.X;
        if (pt.X > result.Right) result.width = pt.X - result.X;
        if (pt.Y < result.Top) result.Y = pt.Y;
        if (pt.Y > result.Bottom) result.height = pt.Y - result.Y;
      }
      return result;
    }

    public static Point GetClosestPtOnSegment(Point offPt,
    Point seg1, Point seg2)
    {
      if (seg1.X == seg2.X && seg1.Y == seg2.Y) return seg1;
      float dx = (seg2.X - seg1.X);
      float dy = (seg2.Y - seg1.Y);
      float q = ((offPt.X - seg1.X) * dx +
        (offPt.Y - seg1.Y) * dy) / ((dx*dx) + (dy*dy));
      if (q < 0) q = 0; else if (q > 1) q = 1;
      return new Point(
        // use MidpointRounding.ToEven in order to explicitly match the nearbyint behaviour on the C++ side
        seg1.X + Mathf.Round(q * dx),
        seg1.Y + Mathf.Round(q * dy)
      );
    }

    public static PointInPolygonResult PointInPolygon(Point pt, PathPoint polygon)
    {
      int len = polygon.Count, start = 0;
      if (len < 3) return PointInPolygonResult.IsOutside;

      while (start < len && polygon[start].Y == pt.Y) start++;
      if (start == len) return PointInPolygonResult.IsOutside;

      bool isAbove = polygon[start].Y < pt.Y, startingAbove = isAbove;
      int val = 0, i = start + 1, end = len;
      while (true)
      {
        if (i == end)
        {
          if (end == 0 || start == 0) break;  
          end = start;
          i = 0;
        }
        
        if (isAbove)
        {
          while (i < end && polygon[i].Y < pt.Y) i++;
        }
        else
        {
          while (i < end && polygon[i].Y > pt.Y) i++;
        }

        if (i == end) continue;

        Point curr = polygon[i], prev;
        if (i > 0) prev = polygon[i - 1];
        else prev = polygon[len - 1];

        if (curr.Y == pt.Y)
        {
          if (curr.X == pt.X || (curr.Y == prev.Y &&
            ((pt.X < prev.X) != (pt.X < curr.X))))
            return PointInPolygonResult.IsOn;
          i++;
          if (i == start) break;
          continue;
        }

        if (pt.X < curr.X && pt.X < prev.X)
        {
          // we're only interested in edges crossing on the left
        }
        else if (pt.X > prev.X && pt.X > curr.X)
        {
          val = 1 - val; // toggle val
        }
        else
        {
          int cps2 = CrossProductSign(prev, curr, pt);
          if (cps2 == 0) return PointInPolygonResult.IsOn;
          if ((cps2 < 0) == isAbove) val = 1 - val;
        }
        isAbove = !isAbove;
        i++;
      }

      if (isAbove == startingAbove) return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;
      if (i == len) i = 0;
      int cps = (i == 0) ?
        CrossProductSign(polygon[len - 1], polygon[0], pt) :
        CrossProductSign(polygon[i - 1], polygon[i], pt);

      if (cps == 0) return PointInPolygonResult.IsOn;
      if ((cps < 0) == isAbove) val = 1 - val;
      return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;
    }

    public static bool Path2ContainsPath1(PathPoint path1, PathPoint path2)
    {
      // we need to make some accommodation for rounding errors
      // so we won't jump if the first vertex is found outside
      PointInPolygonResult pip = PointInPolygonResult.IsOn;
      foreach (Point pt in path1)
      {
        switch (PointInPolygon(pt, path2))
        {
          case PointInPolygonResult.IsOutside:
            if (pip == PointInPolygonResult.IsOutside) return false;
            pip = PointInPolygonResult.IsOutside;
            break;
          case PointInPolygonResult.IsInside:
            if (pip == PointInPolygonResult.IsInside) return true;
            pip = PointInPolygonResult.IsInside;
            break;
          default: break;
        }
      }
      // since path1's location is still equivocal, check its midpoint
      Point mp = GetBounds(path1).MidPoint();
      return InternalClipper.PointInPolygon(mp, path2) != PointInPolygonResult.IsOutside;
    }
  } // InternalClipper

} // namespace
